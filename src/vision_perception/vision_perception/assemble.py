#!/usr/bin/env python3
"""Pick a part and assemble it onto its target, using perception and the
object / gripper / assembly YAML files.

    .venv/bin/python src/vision_perception/vision_perception/assemble.py \
        --assembly plug_into_socket --arm left [--source perception|ground_truth]

Needs the simulation, move_group and (for --source perception) perception_node.

Steps:
  1. scan the part and the target
  2. choose a grasp (all symmetric copies; the first one MoveIt can do)
  3. approach, descend, check arrival
  4. close, lift
  5. re-scan the part in the hand
  6. work out where the part must go (assembly mate frames)
  7. pre-insert, insert (spiral search if blocked), release, retract
  8. re-scan and check the part is seated

Prints a final "RESULT {json}" line for evaluate.py.
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node

from vision_perception.motion import Arm, matrix_to_pose, pose_to_matrix, run_with_abort_retry
from vision_perception.object_registry import Registry


MAX_GRASP_TILT_DEG = 10.0   # more tilt than this: the part has fallen over


def translate(v):
    T = np.eye(4)
    T[:3, 3] = v
    return T


def tilt_deg(T):
    """Angle between the frame's z axis and world vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(T[2, 2])))))


def vertical_tcp(T):
    """Same position and yaw, but pointing exactly straight down.

    The 5-joint arm can point straight down at any yaw, but can't reach
    most slightly tilted poses, so small measured tilts are dropped.
    """
    yaw = math.atan2(T[1, 0], T[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.eye(4)
    out[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ np.diag([1.0, -1.0, -1.0])
    out[:3, 3] = T[:3, 3]
    return out


def rot_angle_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


class Perception:
    """Object poses from /perception/scan, or from Gazebo (ground truth)."""

    def __init__(self, source):
        self.source = source
        self.node = Node("assemble_perception_client")
        self.scans = 0
        if source == "perception":
            from vision_perception_interfaces.srv import Scan
            self._Scan = Scan
            self.client = self.node.create_client(Scan, "/perception/scan")

    def scan(self, names, hints=None, radius=0.05):
        """{name: (4x4 pose, fitness)} for the names found."""
        self.scans += 1
        if self.source == "ground_truth":
            from vision_perception.perception.ground_truth import gz_model_pose
            return {n: (gz_model_pose(n), 1.0) for n in names}
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/perception/scan not available (start perception_node)")
        req = self._Scan.Request(objects=list(names), hint_radius=float(radius))
        if hints:
            req.hints = [matrix_to_pose(hints[n]) for n in names]
        f = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60.0)
        resp = f.result()
        if resp is None:
            raise RuntimeError("/perception/scan timed out")
        print(f"  scan #{self.scans}: {resp.message}", flush=True)
        return {o.name: (pose_to_matrix(o.pose.pose), o.fitness) for o in resp.found}


class StepFailed(RuntimeError):
    """A step failed in a way a fresh attempt (re-scan, re-pick) may fix."""


def yaw_of(T):
    return math.degrees(math.atan2(T[1, 0], T[0, 0]))


def safe_linear(arm, T, label, result):
    """Straight move, or a planned move if the straight line isn't possible.

    Only for moves away from contact, such as retracting.
    """
    try:
        arm.linear(T, label)
    except RuntimeError as e:
        result["recovery"].append(f"{label}: {e} -- planned move instead")
        arm.goto(T, f"{label} (planned)")


def spiral_offsets(radii_mm=(1.5, 3.0, 4.5), per_ring=(6, 6, 8)):
    """(0, 0) first, then rings of offsets in the plane (metres)."""
    out = [np.zeros(2)]
    for k, (r, n) in enumerate(zip(radii_mm, per_ring)):
        for i in range(n):
            a = 2 * math.pi * (i + 0.5 * k) / n
            out.append(np.array([math.cos(a), math.sin(a)]) * r / 1000.0)
    return out


def main_worker():
    ap = argparse.ArgumentParser()
    ap.add_argument("--assembly", default="plug_into_socket")
    ap.add_argument("--arm", default="left", choices=["left", "right"])
    ap.add_argument("--source", default="perception", choices=["perception", "ground_truth"])
    ap.add_argument("--config", default=None, help="config dir (default: installed package)")
    ap.add_argument("--attempts", type=int, default=2, help="full pick+insert attempts (recovery)")
    ap.add_argument("--no-search", action="store_true", help="disable the spiral insertion search")
    ap.add_argument("--insert-error-mm", type=float, nargs=2, default=(0.0, 0.0),
                    help="evaluation: add this XY error (world, mm) to the FIRST insertion goal")
    args, _ = ap.parse_known_args()

    t_start = time.time()
    result = {"assembly": args.assembly, "arm": args.arm, "source": args.source, "success": False,
              "attempts": 0, "recovery": [], "insert_error_mm": list(args.insert_error_mm)}
    reg = Registry(args.config)
    asm = reg.assemblies[args.assembly]
    part, target = reg.objects[asm.part], reg.objects[asm.target]
    grasps = part.grasp_candidates()
    if not grasps:
        raise RuntimeError(f"{part.name} has no grasps in its object file")

    rclpy.init()
    perception = Perception(args.source)
    arm = None
    min_scans = 3                     # scene, in-hand, verify: anything above is a re-scan

    def scan_required(names, hints=None, tries=3):
        for k in range(tries):
            seen = perception.scan(names, hints=hints)
            if all(n in seen for n in names):
                return seen
            missing = [n for n in names if n not in seen]
            result["recovery"].append(f"re-scan: {missing} not found")
            if arm is not None:
                arm.park()        # the arm itself may be hiding them
            time.sleep(0.5)
        raise StepFailed(f"{missing} not found after {tries} scans")

    try:
        insert_error = np.array(args.insert_error_mm) / 1000.0
        seen = scan_required([part.name, target.name])
        T_part, T_target = seen[part.name][0], seen[target.name][0]
        result["first_scan"] = {n: [*np.round(seen[n][0][:3, 3], 5).tolist(), round(yaw_of(seen[n][0]), 2)]
                                for n in (part.name, target.name)}
        arm = Arm(args.arm, reg.grippers[grasps[0].gripper].for_side(args.arm))
        arm.add_object(target, T_target)

        for attempt in range(1, args.attempts + 1):
            result["attempts"] = attempt
            if attempt > 1:
                arm.park()
                seen = scan_required([part.name])
                T_part = seen[part.name][0]
            try:
                xy, dz = attempt_once(arm, reg, asm, part, target, grasps, T_part, T_target,
                                      perception, scan_required, result,
                                      insert_error if attempt == 1 else np.zeros(2), not args.no_search)
            except StepFailed as e:
                result["recovery"].append(f"attempt {attempt} failed: {e}")
                print(f"  attempt {attempt} failed: {e}", flush=True)
                try:
                    arm.open()
                    arm.detach(part.name)
                except Exception:
                    pass
                continue
            result.update(xy_error_mm=round(1000 * xy, 2), z_error_mm=round(1000 * dz, 2))
            print(f"Insertion check: XY {1000 * xy:.1f} mm (tol {1000 * asm.xy_tol:.0f}), "
                  f"Z {1000 * dz:+.1f} mm (must be <= +{1000 * asm.z_tol:.0f})", flush=True)
            if xy <= asm.xy_tol and dz <= asm.z_tol:
                result["success"] = True
                break
            result["recovery"].append(f"attempt {attempt}: not seated (XY {1000 * xy:.1f} mm, Z {1000 * dz:+.1f} mm)")
    except Exception as e:
        result["error"] = str(e)
        raise
    finally:
        result["scans"] = perception.scans
        result["rescans"] = max(0, perception.scans - min_scans - (result["attempts"] - 1) * 3)
        result["time_s"] = round(time.time() - t_start, 1)
        print("RESULT " + json.dumps(result), flush=True)
        perception.node.destroy_node()
    if not result["success"]:
        raise RuntimeError("assembly not completed within tolerance")


def attempt_once(arm, reg, asm, part, target, grasps, T_part, T_target, perception, scan_required,
                 result, insert_error, search):
    """One pick + insert + verify. Returns (xy error, z error) of the part vs its mated pose."""
    arm.add_object(part, T_part)

    # Try the grasp copies with the least wrist rotation first. Near the edge
    # of the reach one copy can fail while another works.
    T_tcp0 = arm.tcp_transform()
    # Grasp straight down (see vertical_tcp): even a 0.5 degree tilt from
    # perception can make a grasp unreachable. A part that has really fallen
    # over can't be grasped from above, so that fails the attempt.
    if tilt_deg(T_part) > MAX_GRASP_TILT_DEG:
        raise StepFailed(f"{part.name} is tilted {tilt_deg(T_part):.0f} deg; can't grasp it from above")
    arm.open()
    g = None
    for cand in sorted(grasps, key=lambda c: rot_angle_deg(T_tcp0[:3, :3].T @ (T_part @ c.tcp_pose)[:3, :3])):
        T_grasp = vertical_tcp(T_part @ cand.tcp_pose)
        T_pre = T_grasp @ translate([0, 0, -cand.pregrasp_offset])    # back along the approach axis
        plan = arm.plan_to_with_line(T_pre, T_grasp, f"pre-grasp {cand.name}")
        if plan is None:
            result["recovery"].append(f"grasp {cand.name}: no approach with a feasible straight descent")
            continue
        arm.execute(plan, "approach part")
        try:
            arm.check_arrived(T_pre, "approach part")
            arm.linear(T_grasp, "descend to grasp")
            g = cand
            break
        except RuntimeError as e:
            result["recovery"].append(f"grasp {cand.name}: {e} -- trying the next grasp")
            print(f"  grasp {cand.name} failed ({e}); trying the next one", flush=True)
            try:
                arm.goto(T_pre, "back to pre-grasp")
            except RuntimeError:
                arm.park()
    if g is None:
        arm.remove_object(part.name)
        raise StepFailed("no grasp could be reached and descended to")
    result["grasp"] = g.name
    arm.close()
    arm.remove_object(part.name)
    try:
        arm.linear(T_pre, "lift")
    except RuntimeError as e:
        # A planned (not straight) lift is fine: the part is clear of the table.
        result["recovery"].append(f"lift: {e} -- planned move instead")
        arm.goto(T_pre, "lift (planned)")

    # Holding something? On nothing, the gripper closes fully.
    if not arm.is_holding():
        raise StepFailed("grasp failed: gripper closed on nothing")

    # Where is the part in the hand? The gripper often hides it from the
    # overhead camera; then use the planned grasp. The spiral search covers
    # the remaining 1-2 mm.
    time.sleep(1.0)                                   # let it stop swinging
    T_tcp = arm.tcp_transform()
    expected = T_tcp @ np.linalg.inv(g.tcp_pose)
    T_held = expected
    result["in_hand"] = "nominal"
    held = perception.scan([part.name], hints={part.name: expected})
    if part.name in held:
        T_meas = held[part.name][0]
        # The real slip is about 1-2 mm. A bigger "measured" offset is a bad
        # scan of a partly hidden part, so it's ignored.
        if np.linalg.norm(T_meas[:3, 3] - expected[:3, 3]) < 0.003 and tilt_deg(T_meas) < 5.0:
            T_held = T_meas
            result["in_hand"] = "measured"
    T_part_tcp = np.linalg.inv(T_held) @ T_tcp
    result["in_hand_offset_mm"] = [round(1000 * v, 1) for v in T_held[:3, 3] - expected[:3, 3]]
    result["in_hand_tilt_deg"] = round(tilt_deg(T_held), 2)
    arm.attach(part, T_held)

    # Mated pose, using the symmetric copy of the target that needs the
    # least rotation of the held part.
    goals = [reg.mated_pose(asm.name, T_target @ np.block([[S, np.zeros((3, 1))], [np.zeros((1, 3)), 1]]))
             for S in target.symmetry.rotations()]
    T_goal = min(goals, key=lambda G: rot_angle_deg(T_held[:3, :3].T @ G[:3, :3]))
    T_tcp_goal = vertical_tcp(T_goal @ T_part_tcp)
    T_tcp_goal[:2, 3] += insert_error                                  # evaluation fault injection
    approach = -asm.approach_axis_world * asm.pre_insert_offset

    # Insert. If the part is blocked (lands on the target and stops short),
    # try a spiral of small offsets.
    plan = arm.plan_to_with_line(translate(approach) @ T_tcp_goal, T_tcp_goal, "pre-insert")
    if plan is None:
        raise StepFailed("no pre-insert configuration with a feasible straight insertion")
    arm.execute(plan, "pre-insert")
    arm.check_arrived(translate(approach) @ T_tcp_goal, "pre-insert")
    inserted = False
    offsets = spiral_offsets() if search else [np.zeros(2)]
    for i, off in enumerate(offsets):
        T_try = T_tcp_goal.copy()
        T_try[:2, 3] += off
        if i > 0:
            arm.linear(translate(approach) @ T_try, f"search {i}: re-approach ({1000 * off[0]:+.1f}, {1000 * off[1]:+.1f}) mm")
        try:
            arm.linear(T_try, f"insert (try {i})", speed=asm.speed_scale)
            inserted = True
            if i > 0:
                result["recovery"].append(f"spiral search: inserted at offset {np.round(1000 * off, 1).tolist()} mm (try {i})")
            break
        except RuntimeError as e:
            print(f"  insert try {i} blocked: {e}", flush=True)
            safe_linear(arm, translate(approach) @ T_try, "back up", result)
    if not inserted:
        result["recovery"].append(f"spiral search: no offset worked ({len(offsets)} tries)")
    arm.open()
    arm.detach(part.name)
    safe_linear(arm, translate(approach) @ vertical_tcp(arm.tcp_transform()), "retract", result)
    # Move out of the camera's view before verifying.
    arm.park()
    if not inserted:
        raise StepFailed("insertion blocked at every search offset")

    final = scan_required([target.name, part.name], hints={target.name: T_target, part.name: T_goal})
    d = final[part.name][0][:3, 3] - T_goal[:3, 3]
    return float(np.hypot(d[0], d[1])), float(d[2])


if __name__ == "__main__":
    sys.exit(run_with_abort_retry(main_worker, os.path.abspath(__file__)))
