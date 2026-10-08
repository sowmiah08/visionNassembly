#!/usr/bin/env python3
"""Randomised evaluation of the perception-driven assembly.

Each trial:
  reset    open the grippers, park the arms, put the part at a random
           position and yaw, maybe hide part of it with the occluder, set
           random sensor noise, pick a random insertion error
  run      assemble.py in a subprocess; read its RESULT line
  score    against Gazebo ground truth, not what the robot believes

Writes one JSON line per trial to <out>/trials.jsonl and a summary to
<out>/summary.json (+ printed).

Needs the stack up (scripts/stack.sh start). Example:
    .venv/bin/python -m vision_perception.evaluate --trials 10 --out /tmp/eval1 \
        --occlusion 0.5 --depth-noise 0.001 --insert-error-mm 3 --seed 1
"""

import argparse
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time

import numpy as np

from vision_perception.object_registry import Registry
from vision_perception.perception.ground_truth import gz_model_pose, visible_pixels

ASSEMBLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assemble.py")


# Below this fraction of its unoccluded pixels the part counts as hidden.
MIN_VISIBLE = 0.10

# Occluder: models/occluder.sdf, a 30 x 30 mm plate that is only visual.
OCCLUDER_Z = 1.01
OCCLUDER_HALF = 0.015


def camera_position(timeout=10.0):
    """World position of the overhead camera's optical centre (from TF)."""
    from rclpy.node import Node
    import rclpy
    import tf2_ros
    from rclpy.time import Time
    node = Node("evaluate_tf")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buf.lookup_transform("world", "camera_optical_frame", Time()).transform.translation
            node.destroy_node()
            return np.array([t.x, t.y, t.z])
        except tf2_ros.TransformException:
            pass
    raise RuntimeError("no TF world -> camera_optical_frame (is the sim running?)")


def occluder_xy(cam, x, y, part_r, part_top, rng):
    """Where to put the occluder so it hides some of the part (about 20-80%).

    The plate hangs between the camera and the part, so it must sit on the
    camera's line of sight to the part: at the plate's height that point is
    c + (p - c) * k, with k = (cz - OCCLUDER_Z) / (cz - part_top). The
    plate's edge is placed across the part's outline there, at a random
    offset and on a random side. visible_fraction records what it really hid.
    """
    k = (cam[2] - OCCLUDER_Z) / (cam[2] - part_top)
    los = cam[:2] + (np.array([x, y]) - cam[:2]) * k
    r_proj = part_r * k
    d = [np.array(v) for v in ((1, 0), (-1, 0), (0, 1), (0, -1))][int(rng.integers(4))]
    s = rng.uniform(-0.4, 0.6) * r_proj
    return tuple(los + d * (OCCLUDER_HALF + s))


def sh(cmd, timeout=30):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def gz_set_pose(name, x, y, z, yaw):
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    sh(["gz", "service", "-s", "/world/default/set_pose", "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req])


def ensure_occluder():
    out = sh(["gz", "model", "--list"]).stdout
    if "occluder" not in out:
        from ament_index_python.packages import get_package_share_directory
        f = os.path.join(get_package_share_directory("so101_description"), "models", "occluder.sdf")
        sh(["ros2", "run", "ros_gz_sim", "create", "-file", f, "-name", "occluder",
            "-x", "5", "-y", "5", "-z", "0"], timeout=60)


def set_param(name, value):
    sh(["ros2", "param", "set", "/perception_node", name, str(float(value))])


def send_joint_goal(controller, joints, positions, seconds=3.0):
    """Joint trajectory straight to a controller (no MoveIt), for resets."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    # A new node name each call; reusing one makes rclpy print warnings.
    send_joint_goal.count = getattr(send_joint_goal, "count", 0) + 1
    node = Node(f"eval_reset_{controller}_{send_joint_goal.count}")
    try:
        c = ActionClient(node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
        if not c.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"{controller} not available")
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = joints
        g.trajectory.points = [JointTrajectoryPoint(positions=[float(p) for p in positions],
                                                    time_from_start=Duration(sec=int(seconds)))]
        f = c.send_goal_async(g)
        rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(node, r, timeout_sec=seconds + 10.0)
    finally:
        node.destroy_node()


def reset_robot():
    arm = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    for side in ("left", "right"):
        send_joint_goal(f"so101_{side}_gripper_controller", [f"{side}_gripper"], [1.2], 1.0)
    for side in ("left", "right"):
        send_joint_goal(f"so101_{side}_arm_controller", [f"{side}_{j}" for j in arm], [0.0] * 5)


def sym_yaw_err(a_deg, b_deg, order):
    period = 360.0 / order
    d = (a_deg - b_deg) % period
    return min(d, period - d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--out", default="/tmp/so101_eval")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--assembly", default="plug_into_socket")
    ap.add_argument("--arm", default="left")
    # Where the part may be placed (world, metres): visible to the overhead
    # camera and reachable for the grasp. These defaults are for the plug.
    ap.add_argument("--x-range", type=float, nargs=2, default=(-0.15, -0.10))
    ap.add_argument("--y-range", type=float, nargs=2, default=(0.00, 0.06))
    ap.add_argument("--occlusion", type=float, default=0.0, help="probability a trial is occluded")
    ap.add_argument("--depth-noise", type=float, default=0.0, help="max depth noise std (m), uniform per trial")
    ap.add_argument("--depth-dropout", type=float, default=0.0, help="max depth dropout fraction")
    ap.add_argument("--rgb-noise", type=float, default=0.0, help="max RGB noise std (0-255)")
    ap.add_argument("--insert-error-mm", type=float, default=0.0, help="max injected insertion XY error (mm)")
    ap.add_argument("--attempts", type=int, default=2)
    ap.add_argument("--timeout", type=float, default=600.0)
    args = ap.parse_args()

    import rclpy
    rclpy.init()
    rng = np.random.default_rng(args.seed)
    reg = Registry()
    asm = reg.assemblies[args.assembly]
    part, target = reg.objects[asm.part], reg.objects[asm.target]
    os.makedirs(args.out, exist_ok=True)
    trials_path = os.path.join(args.out, "trials.jsonl")
    ensure_occluder()
    cam = camera_position()
    verts = np.asarray(part.mesh().vertices)
    part_r = float(np.hypot(verts[:, 0], verts[:, 1]).max())      # footprint radius, part frame
    part_top = 0.761 + float(verts[:, 2].max())                    # top of the part on the table
    T_target_gt = gz_model_pose(target.name)
    T_goal_gt = reg.mated_pose(asm.name, T_target_gt)

    with open(trials_path, "w") as log:
        for k in range(args.trials):
            trial = {"trial": k, "seed": args.seed}
            # ---- randomise ----
            x, y = rng.uniform(*args.x_range), rng.uniform(*args.y_range)
            yaw = rng.uniform(0, 2 * math.pi)
            occluded = bool(rng.random() < args.occlusion)
            noise = {"depth_noise_std": rng.uniform(0, args.depth_noise),
                     "depth_dropout": rng.uniform(0, args.depth_dropout),
                     "rgb_noise_std": rng.uniform(0, args.rgb_noise)}
            err_mag = rng.uniform(0, args.insert_error_mm)
            err_dir = rng.uniform(0, 2 * math.pi)
            ins_err = (err_mag * math.cos(err_dir), err_mag * math.sin(err_dir))
            trial.update(part_xy=[round(x, 4), round(y, 4)], part_yaw_deg=round(math.degrees(yaw), 1),
                         occluded=occluded, noise={k2: round(v, 5) for k2, v in noise.items()},
                         insert_error_mm=[round(v, 2) for v in ins_err])

            # ---- reset ----
            reset_robot()
            gz_set_pose(part.name, x, y, 0.761, yaw)
            # How much of the part the camera can see (from the ground-truth
            # labels), without and with the occluder. This tells a perception
            # miss apart from a part that was simply hidden.
            gz_set_pose("occluder", 5.0, 5.0, 0.0, 0.0)
            time.sleep(1.0)
            px_full = visible_pixels(part.seg_label)
            tg_full = visible_pixels(target.seg_label)
            if occluded:
                ox, oy = occluder_xy(cam, x, y, part_r, part_top, rng)
                gz_set_pose("occluder", ox, oy, OCCLUDER_Z, 0.0)
            else:
                gz_set_pose("occluder", 5.0, 5.0, 0.0, 0.0)
            px_seen = visible_pixels(part.seg_label) if occluded else px_full
            trial["part_pixels"] = px_full
            trial["visible_fraction"] = (round(px_seen / px_full, 3) if px_full and px_seen is not None
                                         else (0.0 if px_full == 0 else None))
            # The target can be hidden too (it's often a few cm away).
            tg_seen = visible_pixels(target.seg_label) if occluded else tg_full
            trial["target_visible_fraction"] = (round(tg_seen / tg_full, 3) if tg_full and tg_seen is not None
                                                else None)
            for n, v in noise.items():
                set_param(n, v)
            time.sleep(1.5)                      # let the part settle on the table
            T_part0 = gz_model_pose(part.name)

            # ---- run ----
            t0 = time.time()
            cmd = [sys.executable, ASSEMBLE, "--assembly", asm.name, "--arm", args.arm,
                   "--attempts", str(args.attempts), "--insert-error-mm", f"{ins_err[0]}", f"{ins_err[1]}"]
            try:
                p = subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout)
                out = p.stdout + p.stderr
                trial["exit_code"] = p.returncode
            except subprocess.TimeoutExpired as e:
                out = (e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or "")
                trial["exit_code"] = "timeout"
            trial["wall_s"] = round(time.time() - t0, 1)
            m = re.findall(r"^RESULT (\{.*\})$", out, flags=re.M)
            res = json.loads(m[-1]) if m else {}
            trial["robot_result"] = res
            with open(os.path.join(args.out, f"trial_{k:03d}.log"), "w") as f:
                f.write(out)

            # ---- score against ground truth ----
            time.sleep(1.0)
            try:
                T_final = gz_model_pose(part.name)
            except RuntimeError as e:
                trial["gt_error"] = str(e)
                log.write(json.dumps(trial) + "\n")
                log.flush()
                print(f"trial {k}: could not read ground truth ({e}) -- skipped", flush=True)
                continue
            dxy = float(np.hypot(*(T_final[:2, 3] - T_goal_gt[:2, 3])))
            dz = float(T_final[2, 3] - T_goal_gt[2, 3])
            trial["gt_success"] = bool(dxy <= asm.xy_tol and dz <= asm.z_tol and dz >= -asm.z_tol - 0.003)
            trial["gt_insert_xy_mm"], trial["gt_insert_z_mm"] = round(1000 * dxy, 2), round(1000 * dz, 2)
            fs = res.get("first_scan", {}).get(part.name)
            if fs:
                trial["pose_err_mm"] = round(1000 * float(np.linalg.norm(np.array(fs[:3]) - T_part0[:3, 3])), 2)
                trial["pose_err_yaw_deg"] = (0.0 if part.symmetry.continuous else  # yaw unobservable
                    round(sym_yaw_err(fs[3], math.degrees(math.atan2(T_part0[1, 0], T_part0[0, 0])),
                                      part.symmetry.order), 2))
            rec = res.get("recovery", [])
            trial["recovery_needed"] = bool(rec)
            trial["recovered"] = bool(rec) and trial["gt_success"]
            trial["rescans"] = res.get("rescans")
            trial["time_s"] = res.get("time_s")
            log.write(json.dumps(trial) + "\n")
            log.flush()
            print(f"trial {k}: part ({x:+.3f},{y:+.3f}) yaw {math.degrees(yaw):5.1f}  occl {int(occluded)} "
                  f"(visible part {trial['visible_fraction']}, target {trial['target_visible_fraction']})  "
                  f"err {err_mag:.1f}mm -> {'SUCCESS' if trial['gt_success'] else 'FAIL'}  "
                  f"insert {trial['gt_insert_xy_mm']:.1f}/{trial['gt_insert_z_mm']:+.1f} mm  "
                  f"pose err {trial.get('pose_err_mm', float('nan')):.1f} mm/{trial.get('pose_err_yaw_deg', float('nan')):.1f} deg  "
                  f"rescans {trial['rescans']}  {trial['time_s']} s  recovery {rec}", flush=True)

    # ---- summary ----
    trials = [t for t in (json.loads(l) for l in open(trials_path)) if "gt_error" not in t]
    def stats(vals):
        vals = [v for v in vals if v is not None]
        if not vals:
            return None
        return {"mean": round(statistics.mean(vals), 2), "median": round(statistics.median(vals), 2),
                "max": round(max(vals), 2), "n": len(vals)}
    ok = [t for t in trials if t["gt_success"]]

    def seen(t):
        return all((t.get(k) is None) or t[k] >= MIN_VISIBLE
                   for k in ("visible_fraction", "target_visible_fraction"))
    needed = [t for t in trials if t["recovery_needed"]]
    summary = {
        "trials": len(trials),
        "assembly_success_rate": round(len(ok) / len(trials), 3) if trials else None,
        "pose_error_mm": stats([t.get("pose_err_mm") for t in trials]),
        "pose_error_yaw_deg": stats([t.get("pose_err_yaw_deg") for t in trials]),
        "insertion_xy_mm_successful": stats([t["gt_insert_xy_mm"] for t in ok]),
        "insertion_z_mm_successful": stats([t["gt_insert_z_mm"] for t in ok]),
        "recovery_needed": len(needed),
        "recovery_success_rate": round(sum(t["recovered"] for t in needed) / len(needed), 3) if needed else None,
        "completion_time_s_successful": stats([t["time_s"] for t in ok]),
        "rescans": stats([t["rescans"] for t in trials]),
        # Occlusion can hide a small part completely, so also report
        # success over the trials where it could be seen.
        "visible_fraction": stats([t.get("visible_fraction") for t in trials]),
        "target_visible_fraction": stats([t.get("target_visible_fraction") for t in trials]),
        # Success over trials where both part and target were visible.
        "success_rate_part_visible": (round(sum(t["gt_success"] for t in vis) / len(vis), 3)
                                      if (vis := [t for t in trials if seen(t)]) else None),
        "trials_part_hidden": sum(1 for t in trials if not seen(t)),
        "config": vars(args),
    }
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    for n in ("depth_noise_std", "depth_dropout", "rgb_noise_std"):
        set_param(n, 0.0)
    gz_set_pose("occluder", 5.0, 5.0, 0.0, 0.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
