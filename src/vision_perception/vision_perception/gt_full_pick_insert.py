#!/usr/bin/env python3
"""Full ground-truth pick-and-insert: picks up the plug using its live
ground-truth pose and inserts it into the socket, with no perception
involved. This is a pre-perception sanity check that the arm is
physically capable of the assembly -- see gt_pick_insert.py for the
Phase 1 (approach-only) version this supersedes.

IMPORTANT CAVEATS, read before running:

1. The 'qos_overrides./clock.subscription.durability' abort (SIGABRT)
   that used to hit MoveItPy + use_sim_time is fixed by pre-declaring
   all four /clock QoS-override parameters in config_dict (see
   run_worker() and PROJECT_LOG.md §8.1). main() still re-execs the
   worker and retries, but only on SIGABRT, as insurance.

2. so101_moveit_config/config/kinematics.yaml has rotation_scale: 0.5
   for left_arm/right_arm (was 0.0, pure position-only IK, which made
   every orientation-constrained goal here unsatisfiable). If insertion
   keeps missing on yaw/tilt, raise it further. The 4-pin pattern is
   square and radially symmetric, so alignment within +/-45 degrees of
   any 90-degree multiple is enough.

3. down_facing_pose()'s roll=pi convention is confirmed for SO-101: the
   gripper's approach axis is +z of *_gripper_frame_link, so roll=pi
   points it straight down (PROJECT_LOG.md §8.4). Note the tip frame
   sits on the FIXED jaw's inner face, not between the jaws -- see the
   GRASP_* constants below and §8.10.

4. The socket's world pose is a hardcoded constant here, not read from
   TF. Since the socket was converted to a standalone, independently
   spawned static SDF model (see PROJECT_LOG.md §6), socket_link no
   longer exists in the URDF/TF tree at all -- there's nothing to look
   up. It never moves during a run, so its known launch-time spawn pose
   (from workcell_gazebo.launch.py) *is* the ground truth.
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


# See caveat 1 above.
MAX_ATTEMPTS = 10

PLUG_POSE_TOPIC = "/ground_truth/plug_pose"
REFERENCE_FRAME = "world"

ARM_GROUP = "left_arm"
TIP_LINK = "left_gripper_frame_link"
GRIPPER_JOINT = "left_gripper"
GRIPPER_CONTROLLER = "so101_left_gripper_controller"
TOUCH_LINKS = ["left_gripper_link", "left_moving_jaw_so101_v1_link"]

# Socket's fixed world pose -- see caveat 4 above. Matches the -x/-y/-z
# spawn args for "socket" in workcell_gazebo.launch.py.
SOCKET_X = 0.0
SOCKET_Y = 0.10
SOCKET_Z = 0.76
SOCKET_YAW = 0.0

# ---- Plug/socket geometry, from assembly_objects.urdf.xacro and plug.sdf ----
PIN_LENGTH = 0.02
BODY_Z = 0.012
PLUG_BODY_SIZE = 0.05
KNOB_ROSETTE_H = 0.004
KNOB_NECK_H = 0.012
KNOB_HEAD_RADIUS = 0.014
SOCKET_BLOCK_HEIGHT = 0.025  # 0.022 m pocket depth + 0.003 m solid floor

# Where the gripper holds the plug. left_gripper_frame_link sits on the
# FIXED jaw's inner face (tip-frame x ~ 0), not between the jaws, and the
# moving jaw closes toward it from -x. The knob head (r=14mm) overhangs the
# neck (r=6mm) by more than the fixed jaw can reach under, so no placement
# lets either jaw touch the neck first (PROJECT_LOG.md §8.13): the knob is
# gripped at the head's equator. During the descent the plug axis sits
# GRASP_AXIS_OFFSET along tip -x so the fixed jaw comes down beside the
# head instead of on top of it.
GRASP_TIP_HEIGHT = 0.050     # tip above the plug's base (head centre is at 0.062)
GRASP_AXIS_OFFSET = 0.016

APPROACH_HEIGHT = 0.03       # the arm can only point straight down up to z~0.85
INSERT_APPROACH_HEIGHT = 0.03  # must stay > PIN_LENGTH-0.002 so pins start above the pocket
# Success check: the plug counts as inserted if its axis is within this of
# the socket's and it isn't sitting higher than seated_z by more than this.
# (Fully seated reads about -2mm: the body rests on the socket top.)
INSERT_XY_TOLERANCE = 0.002
INSERT_Z_TOLERANCE = 0.002
INSERT_SPEED_SCALE = 0.1    # slow final descent
TRANSIT_SPEED_SCALE = 0.3
CARTESIAN_MAX_STEP = 0.005

GRIPPER_OPEN = 1.2
# Joint lower limit. 0.0 still leaves a ~16mm fingertip gap; the moving
# jaw meets the head around +0.19, so commanding the limit squeezes it.
GRIPPER_CLOSED = -0.17
GRIPPER_MOVE_SECONDS = 1.0

# Pause before planning each move so /joint_states catches up with where
# the last trajectory actually ended; planning straight away can start
# from a stale state and trip the 0.01 rad allowed_start_tolerance.
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
    """Snap `yaw` to the closest multiple of 90 degrees from `reference`.

    The plug's 4-pin pattern is square and radially symmetric, so any
    90-degree rotation is an equally valid insertion -- this picks
    whichever one needs the least rotation from the plug's current yaw.
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
    """Two-part collision shape for the plug, in its own frame (z up from
    the pin tips): a box over pins + body + rosette, and a cylinder over
    the neck + knob head. A single fat bounding cylinder (the original
    version) swallowed the space beside the knob where the fixed jaw has
    to go, so every grasp pose counted as a collision.
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
    """Everything moveit_py doesn't provide: the ground-truth pose topic,
    the gripper trajectory action, and the raw /compute_cartesian_path
    service. moveit_py has no Cartesian-path binding at all in this
    Jazzy build (confirmed directly against the installed module), so
    Cartesian segments go through the plain ROS service instead.

    Deliberately NOT setting use_sim_time here: this node never compares
    a message timestamp against its own clock, and having a second node
    in the process alongside MoveItPy's own use_sim_time node was tested
    and does not avoid the known crash anyway (see module docstring and
    PROJECT_LOG.md §7.3) -- so there's no upside to it, only a second
    node to keep track of.
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
        # The constructor binding takes no group, and TOTG refuses a
        # trajectory with none ("planner did not set the group").
        trajectory.joint_model_group_name = ARM_GROUP
        return trajectory

    def _call(self, client, request, timeout_sec=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()

    def _diagnose_cartesian_failure(self, request):
        """/compute_cartesian_path only reports how far it got, not why it
        stopped. Re-run it without collision checking: if that gets further,
        the stop was a collision, so find the first colliding waypoint and
        name the contacts; if not, IK couldn't follow the line."""
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
    # Pre-declare every QoS-override parameter rclcpp's TimeSource creates
    # for the /clock subscription (values = rclcpp::ClockQoS defaults).
    # Otherwise they get declared lazily when the /clock sub is created,
    # and if that happens after MoveIt's TrajectoryExecutionManager has
    # registered its on-set-parameters callback (which rejects every name
    # it doesn't own, with an empty reason), the declare throws and the
    # process aborts. Already-declared params are just read back instead.
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
    # MoveItPy's destructor segfaults ("Deleting MoveItCpp" is the last log
    # line), and it runs as soon as this function's locals are released --
    # turning every run, successful or not, into exit code -11. Holding a
    # module-level reference keeps it alive until main() calls os._exit.
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

        # Tip x axis in world for a down-facing pose at this yaw; the plug
        # axis goes GRASP_AXIS_OFFSET along -x from the tip.
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

        # The world copy of the plug would collide with the gripper from
        # here on; it's re-added below as an attached object instead.
        with psm.read_write() as scene:
            scene.remove_all_collision_objects()

        cartesian_move(pick_approach_pose, "lift plug")

        # Measure where the plug actually ended up in the hand, rather than
        # assuming the planned grasp: closing slides/tilts it, and every
        # later step (the attached collision shape, the insert pose) must
        # use the real tip->plug offset.
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
        # Turn the whole hand about vertical so the held plug's yaw lands on
        # the nearest valid socket orientation, then place the tip so the
        # (rotated) tip->plug offset puts the plug axis on the socket axis.
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
        # Not calling moveit_py_instance.shutdown() either, for the same
        # segfault reason as _KEEP_ALIVE; main() leaves via os._exit.
        helper.destroy_node()
        rclpy.shutdown()


def main():
    if "--worker" in sys.argv:
        # os._exit skips interpreter teardown, which segfaults while
        # destroying MoveItPy and would otherwise hide the real result.
        try:
            run_worker()
        except Exception:
            traceback.print_exc()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(1)
        sys.stdout.flush()
        os._exit(0)

    # Retry driver: re-exec this same file as a fresh subprocess each
    # attempt, since the crash this works around is a process-level
    # abort, not a catchable Python exception -- nothing inside this
    # process can survive it, so the retry has to happen from outside.
    # Works the same whether invoked as `python3 gt_full_pick_insert.py`
    # or via `ros2 run vision_perception gt_full_pick_insert`, since
    # both ultimately call this function.
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"[gt_full_pick_insert] attempt {attempt}/{MAX_ATTEMPTS}")
        result = subprocess.run([sys.executable, __file__, "--worker"])
        if result.returncode == 0:
            break
        if result.returncode != -signal.SIGABRT:
            # Only the qos_overrides abort (SIGABRT) is safe to retry. Any
            # other failure may have left the plug in the gripper or
            # moved, so a fresh attempt would start from the wrong state.
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
