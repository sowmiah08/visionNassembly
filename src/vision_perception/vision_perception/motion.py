"""Motion for either SO-101 arm: planning, straight-line moves, the
gripper, and collision objects. Used by assemble.py.

The arm controllers report success even when the arm is blocked, so every
move checks that the gripper really arrived.
"""

import copy
import os
import signal
import subprocess
import sys
import time
import traceback

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import GetCartesianPath, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

REFERENCE_FRAME = "world"
SETTLE_SECONDS = 0.5
CARTESIAN_MAX_STEP = 0.005
ARRIVE_TOLERANCE = 0.005      # metres; farther than this from the goal = blocked

_KEEP_ALIVE = []


def matrix_to_pose(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    R = T[:3, :3]
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = x, y, z, w
    return p


def pose_to_matrix(p):
    x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def mesh_collision_object(spec, T_world_obj, object_id=None):
    """CollisionObject for an object, from the triangle mesh built from its SDF."""
    m = spec.mesh()
    mesh = Mesh()
    mesh.vertices = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in np.asarray(m.vertices)]
    mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in t]) for t in np.asarray(m.triangles)]
    obj = CollisionObject()
    obj.header.frame_id = REFERENCE_FRAME
    obj.id = object_id or spec.name
    obj.pose = matrix_to_pose(T_world_obj)
    obj.meshes = [mesh]
    obj.mesh_poses = [Pose(orientation=Quaternion(w=1.0))]
    obj.operation = CollisionObject.ADD
    return obj


class _Helper(Node):
    """ROS node for what MoveItPy lacks: straight-line paths, state
    validity checks and the gripper action."""

    def __init__(self, name, gripper_controller):
        super().__init__(name)
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.gripper = ActionClient(self, FollowJointTrajectory, f"/{gripper_controller}/follow_joint_trajectory")
        self.joints = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def joint(self, name, timeout=2.0):
        """Latest measured position of a joint (fresh: spins briefly first)."""
        self.joints.pop(name, None)
        t = time.time()
        while name not in self.joints and time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.joints.get(name)

    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service {client.srv_name} not available (is move_group running?)")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f"{client.srv_name} call timed out (move_group not answering? Try: stack.sh restart)")
        return future.result()


class Arm:
    def __init__(self, side, gripper_spec, node_name="assembler"):
        from moveit.planning import MoveItPy
        from moveit_configs_utils import MoveItConfigsBuilder

        self.side = side
        self.group = f"{side}_arm"
        self.gripper_spec = gripper_spec
        self.tcp = gripper_spec.tcp_link
        self.helper = _Helper(f"{node_name}_helper", f"so101_{side}_gripper_controller")
        self.log = self.helper.get_logger()

        cfg = (MoveItConfigsBuilder("so101_dual_arm_workcell", package_name="so101_moveit_config")
               .planning_pipelines(pipelines=["ompl"])
               .moveit_cpp(file_path="config/moveit_cpp.yaml")
               .to_moveit_configs()).to_dict()
        cfg["use_sim_time"] = True
        # Declare the /clock QoS parameters up front, or MoveItPy can abort.
        cfg["qos_overrides"] = {"/clock": {"subscription": {
            "depth": 1, "durability": "volatile", "history": "keep_last", "reliability": "best_effort"}}}
        self.moveit = MoveItPy(node_name=node_name, config_dict=cfg)
        _KEEP_ALIVE.append(self.moveit)     # MoveItPy crashes when destroyed
        self.pc = self.moveit.get_planning_component(self.group)
        self.model = self.moveit.get_robot_model()
        self.psm = self.moveit.get_planning_scene_monitor()

    # ---- state ----
    def tcp_transform(self):
        self.pc.set_start_state_to_current_state()
        return np.array(self.pc.get_start_state().get_global_link_transform(self.tcp))

    # ---- motion ----
    def plan_to(self, T, label):
        """Plan (not execute) to a TCP pose. Returns the plan or None."""
        time.sleep(SETTLE_SECONDS)
        self.pc.set_start_state_to_current_state()
        goal = PoseStamped()
        goal.header.frame_id = REFERENCE_FRAME
        goal.pose = matrix_to_pose(T)
        self.pc.set_goal_state(pose_stamped_msg=goal, pose_link=self.tcp)
        result = self.pc.plan()
        return result if result else None

    def plan_to_with_line(self, T_pre, T_line_end, label, tries=5):
        """Plan to T_pre so that the straight line T_pre -> T_line_end can be
        followed from where the plan ends. Returns the plan or None.

        Each plan can end in a different arm configuration, and from some of
        them the joints hit their limits partway down the line. So the line
        is tested from the plan's end, and we re-plan until it works.
        """
        from moveit.core.robot_state import robotStateToRobotStateMsg
        for k in range(tries):
            plan = self.plan_to(T_pre, f"{label} (try {k + 1})")
            if plan is None:
                continue
            jt = plan.trajectory.get_robot_trajectory_msg().joint_trajectory
            start = robotStateToRobotStateMsg(self.pc.get_start_state())
            pos = dict(zip(start.joint_state.name, start.joint_state.position))
            pos.update(zip(jt.joint_names, jt.points[-1].positions))
            start.joint_state.name, start.joint_state.position = list(pos), list(pos.values())
            start.joint_state.velocity, start.joint_state.effort = [], []
            req = GetCartesianPath.Request()
            req.header.frame_id = REFERENCE_FRAME
            req.start_state = start
            req.group_name = self.group
            req.link_name = self.tcp
            req.waypoints = [matrix_to_pose(T_line_end)]
            req.max_step = CARTESIAN_MAX_STEP
            req.avoid_collisions = True
            fraction = self.helper.call(self.helper.cartesian, req).fraction
            if fraction >= 0.95:
                return plan
            self.log.info(f"{label}: plan {k + 1} ends in a configuration that can't follow the line "
                          f"({100 * fraction:.0f}%: {self._diagnose(req)}); re-planning")
        return None

    def execute(self, plan, label):
        if not self.moveit.execute(plan.trajectory, controllers=[]):
            raise RuntimeError(f"Execution failed: {label}")

    def goto(self, T, label):
        self.log.info(f"Planning: {label}")
        plan = self.plan_to(T, label)
        if plan is None:
            raise RuntimeError(f"Planning failed: {label}")
        self.execute(plan, label)
        self.check_arrived(T, label)

    def goto_joints(self, positions, label):
        """Joint-space move of this arm's 5 joints."""
        from moveit.core.robot_state import RobotState
        self.log.info(f"Planning: {label}")
        time.sleep(SETTLE_SECONDS)
        self.pc.set_start_state_to_current_state()
        goal = RobotState(self.model)
        goal.set_to_default_values()
        goal.set_joint_group_positions(self.group, np.asarray(positions, float))
        self.pc.set_goal_state(robot_state=goal)
        result = self.pc.plan()
        if not result:
            raise RuntimeError(f"Planning failed: {label}")
        self.execute(result, label)

    def park(self):
        """All joints at zero: the arm stands upright, out of the overhead
        camera's view of the parts."""
        self.goto_joints([0.0] * 5, "park")

    def linear(self, T, label, speed=0.3):
        """Straight-line TCP move to pose T."""
        from moveit.core.robot_state import robotStateToRobotStateMsg
        from moveit.core.robot_trajectory import RobotTrajectory

        self.log.info(f"Cartesian move: {label}")
        time.sleep(SETTLE_SECONDS)
        self.pc.set_start_state_to_current_state()
        start = self.pc.get_start_state()
        req = GetCartesianPath.Request()
        req.header.frame_id = REFERENCE_FRAME
        req.start_state = robotStateToRobotStateMsg(start)
        req.group_name = self.group
        req.link_name = self.tcp
        req.waypoints = [matrix_to_pose(T)]
        req.max_step = CARTESIAN_MAX_STEP
        req.avoid_collisions = True
        resp = self.helper.call(self.helper.cartesian, req)
        if resp.fraction < 0.95:
            raise RuntimeError(f"Cartesian path only {100 * resp.fraction:.0f}% complete ({label}): "
                               f"{self._diagnose(req)}")
        traj = RobotTrajectory(self.model)
        traj.set_robot_trajectory_msg(start, resp.solution)
        traj.joint_model_group_name = self.group     # needed for time parameterization
        if not traj.apply_totg_time_parameterization(speed, speed):
            raise RuntimeError(f"Time parameterization failed: {label}")
        if not self.moveit.execute(traj, controllers=[]):
            raise RuntimeError(f"Execution failed: {label}")
        self.check_arrived(T, label)

    def check_arrived(self, T, label):
        time.sleep(SETTLE_SECONDS)
        err = np.linalg.norm(self.tcp_transform()[:3, 3] - T[:3, 3])
        if err > ARRIVE_TOLERANCE:
            raise RuntimeError(f"Arm stopped {1000 * err:.0f} mm from the goal ({label}): blocked by something")

    def _diagnose(self, req):
        """Why a straight-line path fell short: IK, or which bodies collide."""
        req = copy.deepcopy(req)
        req.avoid_collisions = False
        resp = self.helper.call(self.helper.cartesian, req)
        if resp.fraction < 0.95:
            return "IK could not follow the straight line"
        traj = resp.solution.joint_trajectory
        for point in traj.points:
            state = copy.deepcopy(req.start_state)
            pos = dict(zip(state.joint_state.name, state.joint_state.position))
            pos.update(zip(traj.joint_names, point.positions))
            state.joint_state.name, state.joint_state.position = list(pos), list(pos.values())
            state.joint_state.velocity, state.joint_state.effort = [], []
            v = GetStateValidity.Request(robot_state=state, group_name=self.group)
            r = self.helper.call(self.helper.validity, v)
            if not r.valid:
                return "collision: " + ", ".join(sorted({f"{c.contact_body_1} <-> {c.contact_body_2}"
                                                         for c in r.contacts}))
        return "unknown"

    # ---- gripper ----
    def gripper(self, position, seconds=1.0):
        if not self.helper.gripper.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Gripper action server not available")
        pt = JointTrajectoryPoint(positions=[float(position)],
                                  time_from_start=MsgDuration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.gripper_spec.joint]
        goal.trajectory.points = [pt]
        f = self.helper.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.helper, f, timeout_sec=5.0)
        h = f.result()
        if h is None or not h.accepted:
            raise RuntimeError("Gripper goal rejected")
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self.helper, r, timeout_sec=seconds + 5.0)

    def open(self):
        self.gripper(self.gripper_spec.open)

    def close(self):
        self.gripper(self.gripper_spec.close)

    def is_holding(self):
        """True if the jaws stopped on something instead of closing fully."""
        q = self.helper.joint(self.gripper_spec.joint)
        return q is not None and q > self.gripper_spec.close + self.gripper_spec.held_margin

    # ---- planning scene ----
    def add_object(self, spec, T):
        with self.psm.read_write() as scene:
            scene.apply_collision_object(mesh_collision_object(spec, T))

    def remove_object(self, name):
        obj = CollisionObject(id=name, operation=CollisionObject.REMOVE)
        obj.header.frame_id = REFERENCE_FRAME
        with self.psm.read_write() as scene:
            scene.apply_collision_object(obj)

    def attach(self, spec, T_world_obj):
        a = AttachedCollisionObject(link_name=self.tcp, object=mesh_collision_object(spec, T_world_obj),
                                    touch_links=self.gripper_spec.touch_links)
        with self.psm.read_write() as scene:
            scene.process_attached_collision_object(a)

    def detach(self, name):
        a = AttachedCollisionObject(link_name=self.tcp)
        a.object.id = name
        a.object.operation = CollisionObject.REMOVE
        with self.psm.read_write() as scene:
            scene.process_attached_collision_object(a)


def run_with_abort_retry(worker, script_path, max_attempts=10):
    """Run `worker()` in a new process; retry only if it aborts (SIGABRT).

    Any other failure is not retried: the parts may already have moved.
    The worker exits with os._exit, so MoveItPy is never destroyed.
    """
    if "--worker" in sys.argv:
        try:
            worker()
        except Exception:
            traceback.print_exc()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(1)
        sys.stdout.flush()
        os._exit(0)
    for attempt in range(1, max_attempts + 1):
        print(f"[{os.path.basename(script_path)}] attempt {attempt}/{max_attempts}", flush=True)
        rc = subprocess.run([sys.executable, script_path, "--worker"] + sys.argv[1:]).returncode
        if rc == 0:
            return 0
        if rc != -signal.SIGABRT:
            print(f"[{os.path.basename(script_path)}] failed (exit code {rc}), not retrying.", flush=True)
            return 1
        print(f"[{os.path.basename(script_path)}] aborted (SIGABRT), retrying...", flush=True)
    return 1
