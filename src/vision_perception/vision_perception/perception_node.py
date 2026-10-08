#!/usr/bin/env python3
"""Perception as a ROS 2 service: /perception/scan
(vision_perception_interfaces/srv/Scan).

Loads SAM 2 once, then answers scan requests with object poses and a
confidence. Each request uses a camera frame taken after the request
arrived, so a pose never comes from before the robot's last move. Poses
are also published on /perception/<name>/pose, and a mask overlay on
/perception/debug_image.

Run it with the venv:
    .venv/bin/python -m vision_perception.perception_node --ros-args -p use_sim_time:=true
"""

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros

from vision_perception.object_registry import Registry
from vision_perception.perception.pipeline import ScanPipeline
from vision_perception.perception.robot_filter import RobotFilter
from vision_perception.perception.segmenter import Sam2Segmenter
from vision_perception.scan_eval import image_to_array, quat_to_matrix
from vision_perception_interfaces.msg import ObjectPose
from vision_perception_interfaces.srv import Scan

CAM = "/overhead_camera"


def matrix_to_pose(T, msg):
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in T[:3, 3])
    R = T[:3, :3]
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = x, y, z, w
    return msg


def pose_to_matrix(p):
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(p.orientation)
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.min_fitness = self.declare_parameter("min_fitness", 0.8).value
        self.settle = self.declare_parameter("settle_sec", 0.3).value
        self.frame_timeout = self.declare_parameter("frame_timeout_sec", 5.0).value
        # Simulated noise for evaluation (0 = off). Read on every scan, so
        # `ros2 param set` can change it between trials.
        self.declare_parameter("depth_noise_std", 0.0)     # metres, Gaussian per pixel
        self.declare_parameter("depth_dropout", 0.0)       # fraction of depth pixels set invalid
        self.declare_parameter("rgb_noise_std", 0.0)       # 0-255 intensity units, Gaussian
        self._rng = np.random.default_rng()

        self._lock = threading.Lock()
        self._latest = {}
        sensors = ReentrantCallbackGroup()
        for key, topic, typ in (("rgb", f"{CAM}/rgb/image_raw", Image),
                                ("depth", f"{CAM}/depth/image_raw", Image),
                                ("info", f"{CAM}/rgb/camera_info", CameraInfo)):
            self.create_subscription(typ, topic, lambda m, k=key: self._store(k, m), 5,
                                     callback_group=sensors)
        self._urdf = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", lambda m: setattr(self, "_urdf", m.data),
                                 latched, callback_group=sensors)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.registry = Registry()
        self.get_logger().info(f"Objects: {list(self.registry.objects)}; loading SAM 2...")
        self.segmenter = Sam2Segmenter()
        self.pipeline = None   # needs the URDF; built on the first request
        self.pose_pubs = {n: self.create_publisher(PoseStamped, f"/perception/{n}/pose", 10)
                          for n in self.registry.objects}
        self.debug_pub = self.create_publisher(Image, "/perception/debug_image", 2)
        self.create_service(Scan, "/perception/scan", self._on_scan,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("Ready: call /perception/scan")

    def _store(self, key, msg):
        with self._lock:
            self._latest[key] = msg

    def _now_sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh_frame(self, after):
        """RGB + depth + info stamped after `after` (sim seconds) and within 0.1 s of each other."""
        t0 = time.time()
        while time.time() - t0 < self.frame_timeout:
            with self._lock:
                f = dict(self._latest)
            if all(k in f for k in ("rgb", "depth", "info")):
                s_rgb, s_depth = stamp_sec(f["rgb"]), stamp_sec(f["depth"])
                if min(s_rgb, s_depth) >= after and abs(s_rgb - s_depth) < 0.1:
                    return f
            time.sleep(0.02)
        raise RuntimeError("no fresh, synchronised RGB-D frame (is the simulation running?)")

    def _link_transform(self, link):
        try:
            tf = self.tf_buffer.lookup_transform("world", link, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(tf.transform.rotation)
        tr = tf.transform.translation
        T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

    def _on_scan(self, req, resp):
        t0 = time.time()
        wanted = list(req.objects) or list(self.registry.objects)
        unknown = [n for n in wanted if n not in self.registry.objects]
        if unknown:
            resp.success, resp.message, resp.missing = False, f"unknown objects: {unknown}", unknown
            return resp
        if req.hints and len(req.hints) != len(req.objects):
            resp.success, resp.message, resp.missing = False, "hints must match objects one-to-one", wanted
            return resp
        hints = {n: pose_to_matrix(h) for n, h in zip(req.objects, req.hints)}
        try:
            if self.pipeline is None:
                if self._urdf is None:
                    raise RuntimeError("no URDF on /robot_description yet")
                self.pipeline = ScanPipeline(self.registry, self.segmenter, RobotFilter(self._urdf))
            frame = self._fresh_frame(after=self._now_sim() + self.settle)
            T_cam = self._link_transform(frame["rgb"].header.frame_id)
            if T_cam is None:
                raise RuntimeError(f"no TF world -> {frame['rgb'].header.frame_id}")
            K = np.array(frame["info"].k, float).reshape(3, 3)
            rgb = image_to_array(frame["rgb"])[:, :, :3]
            depth = image_to_array(frame["depth"])
            rgb, depth = self._add_noise(rgb, depth)
            res = self.pipeline.run(rgb, depth, K, T_cam, link_transform=self._link_transform,
                                    objects=wanted, hints=hints,
                                    hint_radius=req.hint_radius or 0.05)
        except Exception as e:  # report, don't crash the service
            resp.success, resp.message, resp.missing = False, f"scan failed: {e}", wanted
            resp.duration = float(time.time() - t0)
            self.get_logger().error(resp.message)
            return resp

        found = res.by_name()
        header = frame["rgb"].header
        header.frame_id = "world"
        why = {}                       # missing object -> reason, for the log / message
        for name in wanted:
            d = found.get(name)
            if d is None or d.T is None or d.fitness < self.min_fitness:
                resp.missing.append(name)
                why[name] = ("no blob identified as it" if d is None else
                             "pose fit failed" if d.T is None else
                             f"fit {d.fitness:.2f} < {self.min_fitness}")
                continue
            op = ObjectPose(name=name, fitness=float(d.fitness), rmse=float(d.rmse),
                            n_points=int(d.n_points), mask_score=float(d.mask_score))
            op.pose.header = header
            matrix_to_pose(d.T, op.pose.pose)
            resp.found.append(op)
            self.pose_pubs[name].publish(op.pose)
        resp.success = not resp.missing
        resp.duration = float(time.time() - t0)
        resp.message = (f"found {[o.name for o in resp.found]}, missing {list(resp.missing)}"
                        + "".join(f" ({n}: {r})" for n, r in why.items()) + "; "
                        + ", ".join(f"{k} {1000 * v:.0f} ms" for k, v in res.timing.items()))
        if why:
            # Sizes of blobs no object claimed, to show why one was missed.
            spare = [d.signature for d in res.detections if d.name is None and d.signature is not None]
            resp.message += "; unclaimed blobs h/L/W mm: " + ", ".join(
                "/".join(f"{1000 * v:.0f}" for v in sig) for sig in spare)
        self.get_logger().info(resp.message)
        self._publish_debug(rgb, res, frame["rgb"].header)
        return resp

    def _add_noise(self, rgb, depth):
        """Simulated sensor noise (evaluation only)."""
        dn = self.get_parameter("depth_noise_std").value
        dd = self.get_parameter("depth_dropout").value
        rn = self.get_parameter("rgb_noise_std").value
        if dn > 0:
            depth = depth + self._rng.normal(0.0, dn, depth.shape).astype(np.float32)
        if dd > 0:
            depth = np.where(self._rng.random(depth.shape) < dd, np.nan, depth).astype(np.float32)
        if rn > 0:
            rgb = np.clip(rgb.astype(np.float32) + self._rng.normal(0.0, rn, rgb.shape), 0, 255).astype(np.uint8)
        return rgb, depth

    def _publish_debug(self, rgb, res, header):
        img = rgb.copy()
        for i, d in enumerate(res.detections):
            c = np.array([(255, 0, 255), (0, 255, 0), (255, 128, 0), (0, 200, 255)][i % 4], np.uint8)
            img[d.mask] = (0.45 * img[d.mask] + 0.55 * c).astype(np.uint8)
        msg = Image(header=header, height=img.shape[0], width=img.shape[1], encoding="rgb8",
                    step=img.shape[1] * 3, data=img.tobytes())
        self.debug_pub.publish(msg)


def main():
    rclpy.init()
    node = PerceptionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
