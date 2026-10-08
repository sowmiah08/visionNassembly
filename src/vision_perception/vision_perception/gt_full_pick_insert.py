#!/usr/bin/env python3
"""Pick the plug and insert it into the socket using the TRUE plug pose
from Gazebo (no perception). This checks that the arm can physically do
the task.

The socket pose is a constant: it's a static model spawned by
workcell_gazebo.launch.py and never moves.
"""

import copy
import math
import os
import signal
import subprocess
import sys
import time
import traceback

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import GetCartesianPath, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit.core.robot_state import robotStateToRobotStateMsg
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


# The worker is retried only if it aborts (see main()).
MAX_ATTEMPTS = 10

PLUG_POSE_TOPIC = "/ground_truth/plug_pose"
REFERENCE_FRAME = "world"

ARM_GROUP = "left_arm"
TIP_LINK = "left_gripper_frame_link"
GRIPPER_JOINT = "left_gripper"
GRIPPER_CONTROLLER = "so101_left_gripper_controller"
TOUCH_LINKS = ["left_gripper_link", "left_moving_jaw_so101_v1_link"]

# Socket pose: must match its spawn position in workcell_gazebo.launch.py.
SOCKET_X = 0.0
SOCKET_Y = 0.10
SOCKET_Z = 0.76
SOCKET_YAW = 0.0

# Plug and socket sizes (from plug.sdf / socket.sdf), metres.
PIN_LENGTH = 0.02
BODY_Z = 0.012
PLUG_BODY_SIZE = 0.05
KNOB_ROSETTE_H = 0.004
KNOB_NECK_H = 0.012
KNOB_HEAD_RADIUS = 0.014
SOCKET_BLOCK_HEIGHT = 0.025  # 0.022 m pocket depth + 0.003 m solid floor

# Grasp: the tip frame sits on the fixed jaw, not between the jaws, so the
# tip is placed GRASP_AXIS_OFFSET to the side of the plug. The fixed jaw then
# comes down beside the knob, and the jaws clamp the knob at its widest part.
GRASP_TIP_HEIGHT = 0.050     # tip above the plug's base (head sphere centre is at 0.048)
GRASP_AXIS_OFFSET = 0.016

APPROACH_HEIGHT = 0.03       # the arm can only point straight down up to z ~0.85
INSERT_APPROACH_HEIGHT = 0.03  # pins must start above the pockets
# Inserted = plug axis within XY tolerance of the socket and not higher than
# seated_z + Z tolerance. Fully seated reads about -2 mm.
INSERT_XY_TOLERANCE = 0.002
INSERT_Z_TOLERANCE = 0.002
INSERT_SPEED_SCALE = 0.1    # slow final descent
TRANSIT_SPEED_SCALE = 0.3
CARTESIAN_MAX_STEP = 0.005

GRIPPER_OPEN = 1.2
GRIPPER_CLOSED = -0.17       # joint limit, so the jaws keep squeezing the knob
GRIPPER_MOVE_SECONDS = 1.0

# Wait before planning, so the joint states are up to date. Otherwise the
# plan can start from an old state and MoveIt refuses to run it.
SETTLE_SECONDS = 0.5

DOWN_ROLL = math.pi
DOWN_PITCH = 0.0


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_from_quat(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def nearest_quarter_turn(yaw, reference):
    """Closest yaw to `yaw` that is `reference` plus a multiple of 90 degrees.

    The 4 pins form a square, so any 90-degree turn fits the socket.
    """
    n = round((yaw - reference) / (math.pi / 2))
    return reference + n * (math.pi / 2)


def down_facing_pose(x, y, z, yaw):
    qx, qy, qz, qw = quat_from_rpy(DOWN_ROLL, DOWN_PITCH, yaw)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def build_plug_collision_object(pose_stamped, object_id="plug"):
    """Plug collision shape: a box for pins + body, a cylinder for the knob.

    It must stay tight around the knob, because the fixed jaw goes beside it.
    """
    lower_h = PIN_LENGTH + BODY_Z + KNOB_ROSETTE_H            # 0.036
    knob_h = KNOB_NECK_H + 2 * KNOB_HEAD_RADIUS               # 0.040

    lower = SolidPrimitive()
    lower.type = SolidPrimitive.BOX
    lower.dimensions = [PLUG_BODY_SIZE, PLUG_BODY_SIZE, lower_h]
    lower_pose = Pose()
    lower_pose.position.z = lower_h / 2
    lower_pose.orientation.w = 1.0

    knob = SolidPrimitive()
    knob.type = SolidPrimitive.CYLINDER
    knob.dimensions = [knob_h, KNOB_HEAD_RADIUS]
    knob_pose = Pose()
    knob_pose.position.z = lower_h + knob_h / 2
    knob_pose.orientation.w = 1.0

    obj = CollisionObject()
    obj.header = pose_stamped.header
    obj.id = object_id
    obj.pose = pose_stamped.pose
    obj.primitives = [lower, knob]
    obj.primitive_poses = [lower_pose, knob_pose]
    obj.operation = CollisionObject.ADD
    return obj


class GtPickInsertHelper(Node):
    """What MoveItPy doesn't provide: the ground-truth plug pose, the
    gripper action, and straight-line (Cartesian) paths, which MoveItPy has
    no Python API for, so they go through move_group's service.
    """

    def __init__(self):
        super().__init__("gt_pick_insert_helper")
        self._plug_pose = None
        self.create_subscription(PoseStamped, PLUG_POSE_TOPIC, self._on_plug_pose, 10)

        self._cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, f"/{GRIPPER_CONTROLLER}/follow_joint_trajectory"
        )

    def _on_plug_pose(self, msg):
        self._plug_pose = msg

    def wait_for_plug_pose(self, timeout_sec=10.0):
        self._plug_pose = None
        deadline_spins = int(timeout_sec / 0.1)
        for _ in range(deadline_spins):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._plug_pose is not None:
                return self._plug_pose
        raise RuntimeError("Timed out waiting for the ground-truth plug pose topic")

    def move_gripper(self, position, duration_sec=GRIPPER_MOVE_SECONDS):
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Gripper action server not available")

        point = JointTrajectoryPoint()
        point.positions = [position]
        sec = int(duration_sec)
        point.time_from_start = MsgDuration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        goal.trajectory.points = [point]

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Gripper goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)

    def compute_cartesian_path(self, robot_model, robot_state, waypoints):
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/compute_cartesian_path service not available")

        request = GetCartesianPath.Request()
        request.header.frame_id = REFERENCE_FRAME
        request.start_state = robotStateToRobotStateMsg(robot_state)
        request.group_name = ARM_GROUP
        request.link_name = TIP_LINK
        request.waypoints = waypoints
        request.max_step = CARTESIAN_MAX_STEP
        request.avoid_collisions = True

        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            raise RuntimeError("compute_cartesian_path call failed")
        if response.fraction < 0.95:
            reason = self._diagnose_cartesian_failure(request)
            raise RuntimeError(
                f"Cartesian path only {response.fraction * 100:.0f}% complete: {reason}")

        trajectory = RobotTrajectory(robot_model)
        trajectory.set_robot_trajectory_msg(robot_state, response.solution)
        # Time parameterization needs the group, and the constructor can't set it.
        trajectory.joint_model_group_name = ARM_GROUP
        return trajectory

    def _call(self, client, request, timeout_sec=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()

    def _diagnose_cartesian_failure(self, request):
        """Say why a straight-line path stopped: IK, or which bodies collide.

        The service only says how far it got. If the path completes without
        collision checking, the cause was a collision, and the first invalid
        waypoint names the bodies.
        """
        request.avoid_collisions = False
        response = self._call(self._cartesian_client, request)
        if response is None or response.fraction < 0.95:
            return "IK could not follow the straight line (fails even without collision checking)"
        if not self._validity_client.wait_for_service(timeout_sec=5.0):
            return "collision (no /check_state_validity service to say which)"
        traj = response.solution.joint_trajectory
        for point in traj.points:
            state = copy.deepcopy(request.start_state)
            positions = dict(zip(state.joint_state.name, state.joint_state.position))
            positions.update(zip(traj.joint_names, point.positions))
            state.joint_state.name = list(positions)
            state.joint_state.position = list(positions.values())
            state.joint_state.velocity = []
            state.joint_state.effort = []
            validity = GetStateValidity.Request()
            validity.robot_state = state
            validity.group_name = ARM_GROUP
            result = self._call(self._validity_client, validity)
            if result is not None and not result.valid:
                pairs = sorted({f"{c.contact_body_1} <-> {c.contact_body_2}" for c in result.contacts})
                return "collision: " + (", ".join(pairs) or "invalid state, no contacts reported")
        return "unknown (the collision-free retry succeeded and every waypoint is valid)"


_KEEP_ALIVE = []


def run_worker():
    rclpy.init()
    helper = GtPickInsertHelper()
    log = helper.get_logger()

    moveit_config = (
        MoveItConfigsBuilder("so101_dual_arm_workcell", package_name="so101_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )
    config_dict = moveit_config.to_dict()
    config_dict["use_sim_time"] = True
    # Declare the /clock QoS parameters up front (default values). If they
    # are declared later, MoveIt's parameter callback rejects them and the
    # process aborts at random.
    config_dict["qos_overrides"] = {
        "/clock": {
            "subscription": {
                "depth": 1,
                "durability": "volatile",
                "history": "keep_last",
                "reliability": "best_effort",
            }
        }
    }

    moveit_py_instance = MoveItPy(node_name="gt_full_pick_insert", config_dict=config_dict)
    # MoveItPy crashes when it's destroyed. Keep a reference so it never is;
    # main() exits with os._exit.
    _KEEP_ALIVE.append(moveit_py_instance)
    arm = moveit_py_instance.get_planning_component(ARM_GROUP)
    robot_model = moveit_py_instance.get_robot_model()
    psm = moveit_py_instance.get_planning_scene_monitor()

    def current_tip_transform():
        arm.set_start_state_to_current_state()
        return arm.get_start_state().get_global_link_transform(TIP_LINK)

    def goto_pose(pose, label):
        log.info(f"Planning: {label}")
        time.sleep(SETTLE_SECONDS)
        arm.set_start_state_to_current_state()
        goal = PoseStamped()
        goal.header.frame_id = REFERENCE_FRAME
        goal.pose = pose
        arm.set_goal_state(pose_stamped_msg=goal, pose_link=TIP_LINK)
        result = arm.plan()
        if not result:
            raise RuntimeError(f"Planning failed: {label}")
        status = moveit_py_instance.execute(result.trajectory, controllers=[])
        if not status:
            raise RuntimeError(f"Execution failed: {label} ({status})")

    def cartesian_move(target_pose, label, velocity_scale=TRANSIT_SPEED_SCALE):
        log.info(f"Cartesian move: {label}")
        time.sleep(SETTLE_SECONDS)
        arm.set_start_state_to_current_state()
        start_state = arm.get_start_state()
        trajectory = helper.compute_cartesian_path(robot_model, start_state, [target_pose])
        if not trajectory.apply_totg_time_parameterization(velocity_scale, velocity_scale):
            raise RuntimeError(f"Time parameterization failed: {label}")
        status = moveit_py_instance.execute(trajectory, controllers=[])
        if not status:
            raise RuntimeError(f"Execution failed: {label} ({status})")

    try:
        log.info("Waiting for ground-truth plug pose...")
        plug_pose_msg = helper.wait_for_plug_pose()

        plug_x = plug_pose_msg.pose.position.x
        plug_y = plug_pose_msg.pose.position.y
        plug_z = plug_pose_msg.pose.position.z
        plug_yaw = yaw_from_quat(
            plug_pose_msg.pose.orientation.x, plug_pose_msg.pose.orientation.y,
            plug_pose_msg.pose.orientation.z, plug_pose_msg.pose.orientation.w)

        # Tip x axis in the world for a down-facing pose at this yaw.
        grasp_dir_x, grasp_dir_y = math.cos(plug_yaw), math.sin(plug_yaw)
        grasp_x = plug_x + GRASP_AXIS_OFFSET * grasp_dir_x
        grasp_y = plug_y + GRASP_AXIS_OFFSET * grasp_dir_y
        grasp_z = plug_z + GRASP_TIP_HEIGHT
        pick_approach_pose = down_facing_pose(grasp_x, grasp_y, grasp_z + APPROACH_HEIGHT, plug_yaw)
        pick_grasp_pose = down_facing_pose(grasp_x, grasp_y, grasp_z, plug_yaw)

        socket_top_z = SOCKET_Z + SOCKET_BLOCK_HEIGHT
        seated_z = socket_top_z - PIN_LENGTH + 0.002  # plug base when seated

        # ---- Pick ----
        helper.move_gripper(GRIPPER_OPEN)

        with psm.read_write() as scene:
            scene.apply_collision_object(build_plug_collision_object(plug_pose_msg))

        goto_pose(pick_approach_pose, "approach plug")
        cartesian_move(pick_grasp_pose, "descend to plug")
        reached_z = current_tip_transform()[2, 3]
        if reached_z - grasp_z > 0.005:
            raise RuntimeError(
                f"Gripper stopped {1000 * (reached_z - grasp_z):.0f} mm above the grasp height "
                f"(something under the jaws is in the way)")
        helper.move_gripper(GRIPPER_CLOSED)

        # Remove the plug from the scene; it's added back attached to the gripper.
        with psm.read_write() as scene:
            scene.remove_all_collision_objects()

        cartesian_move(pick_approach_pose, "lift plug")

        # Measure where the plug really is in the hand: closing the gripper
        # moves it a little, and the insert must use the real offset.
        time.sleep(SETTLE_SECONDS)
        held_msg = helper.wait_for_plug_pose()
        held = held_msg.pose
        if held.position.z < plug_z + 0.01:
            raise RuntimeError(
                f"Grasp failed: plug still on the table after lift (z={held.position.z:.3f})")
        tip_T = current_tip_transform()
        tip_yaw = math.atan2(tip_T[1, 0], tip_T[0, 0])
        held_yaw = yaw_from_quat(held.orientation.x, held.orientation.y,
                                 held.orientation.z, held.orientation.w)
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0,
            1 - 2 * (held.orientation.x ** 2 + held.orientation.y ** 2)))))
        off_x = held.position.x - tip_T[0, 3]
        off_y = held.position.y - tip_T[1, 3]
        off_z = held.position.z - tip_T[2, 3]
        log.info(f"Plug in hand: offset from tip (world) = ({1000 * off_x:.1f}, {1000 * off_y:.1f}, "
                 f"{1000 * off_z:.1f}) mm, tilt = {tilt_deg:.1f} deg")

        attach = AttachedCollisionObject()
        attach.link_name = TIP_LINK
        attach.object = build_plug_collision_object(held_msg)
        attach.touch_links = TOUCH_LINKS
        with psm.read_write() as scene:
            scene.process_attached_collision_object(attach)

        # ---- Insert ----
        # Turn the hand so the plug lines up with the socket, then place the
        # tip so the plug's axis lands on the socket's axis.
        delta = nearest_quarter_turn(held_yaw, SOCKET_YAW) - held_yaw
        rot_x = math.cos(delta) * off_x - math.sin(delta) * off_y
        rot_y = math.sin(delta) * off_x + math.cos(delta) * off_y
        insert_tip_x = SOCKET_X - rot_x
        insert_tip_y = SOCKET_Y - rot_y
        insert_tip_z = seated_z - off_z
        insert_approach_pose = down_facing_pose(
            insert_tip_x, insert_tip_y, insert_tip_z + INSERT_APPROACH_HEIGHT, tip_yaw + delta)
        insert_seated_pose = down_facing_pose(
            insert_tip_x, insert_tip_y, insert_tip_z, tip_yaw + delta)

        goto_pose(insert_approach_pose, "approach socket")
        cartesian_move(insert_seated_pose, "insert into socket", velocity_scale=INSERT_SPEED_SCALE)
        helper.move_gripper(GRIPPER_OPEN)

        detach = AttachedCollisionObject()
        detach.link_name = TIP_LINK
        detach.object.id = "plug"
        detach.object.operation = CollisionObject.REMOVE
        with psm.read_write() as scene:
            scene.process_attached_collision_object(detach)
            scene.remove_all_collision_objects()

        cartesian_move(insert_approach_pose, "retract")

        # ---- Success check ----
        log.info("Re-reading ground-truth plug pose for a success check...")
        final_pose = helper.wait_for_plug_pose()
        dx = final_pose.pose.position.x - SOCKET_X
        dy = final_pose.pose.position.y - SOCKET_Y
        dz = final_pose.pose.position.z - seated_z
        xy_error_mm = math.hypot(dx, dy) * 1000
        z_error_mm = dz * 1000
        log.info(
            f"Result: XY error = {xy_error_mm:.1f} mm, Z error = {z_error_mm:.1f} mm "
            f"(pass threshold: ~2mm XY, seated in Z)")
        if xy_error_mm > INSERT_XY_TOLERANCE * 1000 or z_error_mm > INSERT_Z_TOLERANCE * 1000:
            raise RuntimeError("Insertion check failed: plug is not seated in the socket")

    finally:
        # No moveit_py_instance.shutdown(): it crashes (see _KEEP_ALIVE).
        helper.destroy_node()
        rclpy.shutdown()


def main():
    if "--worker" in sys.argv:
        # os._exit skips Python's cleanup, which would crash in MoveItPy.
        try:
            run_worker()
        except Exception:
            traceback.print_exc()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(1)
        sys.stdout.flush()
        os._exit(0)

    # Run the worker in a new process each attempt, because the abort it
    # guards against kills the whole process.
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"[gt_full_pick_insert] attempt {attempt}/{MAX_ATTEMPTS}")
        result = subprocess.run([sys.executable, __file__, "--worker"])
        if result.returncode == 0:
            break
        if result.returncode != -signal.SIGABRT:
            # Only an abort is safe to retry: after any other failure the
            # plug may already be moved or in the gripper.
            print(f"[gt_full_pick_insert] attempt {attempt} failed "
                  f"(exit code {result.returncode}), not retrying.")
            sys.exit(1)
        print(
            f"[gt_full_pick_insert] attempt {attempt} aborted "
            f"(exit code {result.returncode}), retrying..."
        )
    else:
        print(f"[gt_full_pick_insert] gave up after {MAX_ATTEMPTS} attempts.")
        sys.exit(1)


if __name__ == "__main__":
    main()
