#!/usr/bin/env python3
"""One scan of the table, scored against ground truth (for testing).

Grabs one RGB + depth + label frame, runs the perception pipeline, and
prints each object's mask IoU and pose error. Saves an overlay image.

Run with the venv:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    .venv/bin/python -m vision_perception.scan_eval [--out scan.png]
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros

from vision_perception.object_registry import Registry
from vision_perception.perception.ground_truth import gz_model_pose
from vision_perception.perception.pipeline import ScanPipeline
from vision_perception.perception.pose import pose_error
from vision_perception.perception.robot_filter import RobotFilter
from vision_perception.perception.segmenter import Sam2Segmenter

CAM = "/overhead_camera"
TOPICS = {
    "rgb": f"{CAM}/rgb/image_raw",
    "depth": f"{CAM}/depth/image_raw",
    "labels": f"{CAM}/segmentation/labels_map",
}


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def image_to_array(msg):
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1).copy()


class FrameGrabber(Node):
    def __init__(self):
        super().__init__("scan_eval")
        self.msgs = {}
        for key, topic in TOPICS.items():
            self.create_subscription(Image, topic, lambda m, k=key: self.msgs.__setitem__(k, m), 5)
        self.create_subscription(CameraInfo, f"{CAM}/rgb/camera_info",
                                 lambda m: self.msgs.__setitem__("info", m), 5)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.urdf = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description",
                                 lambda m: setattr(self, "urdf", m.data), latched)

    def wait_for_urdf(self, timeout=10.0):
        t0 = time.time()
        while self.urdf is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.urdf is None:
            raise RuntimeError("No URDF on /robot_description (is robot_state_publisher running?)")
        return self.urdf

    def link_transform(self, link):
        try:
            tf = self.tf_buffer.lookup_transform("world", link, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(tf.transform.rotation)
        tr = tf.transform.translation
        T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

    def grab(self, timeout=10.0, settle=1.0):
        """Latest frame of each stream, all within 0.1 s of each other (sim time)."""
        t0 = time.time()
        self.msgs.clear()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.msgs) == 4 and time.time() - t0 > settle:
                stamps = [self.msgs[k].header.stamp.sec + self.msgs[k].header.stamp.nanosec * 1e-9
                          for k in TOPICS]
                if max(stamps) - min(stamps) < 0.1:
                    break
        else:
            raise RuntimeError(f"Missing or unsynchronised camera topics: have {sorted(self.msgs)}")
        frame_id = self.msgs["rgb"].header.frame_id
        tf = self.tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time(),
                                             timeout=rclpy.duration.Duration(seconds=2.0))
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(tf.transform.rotation)
        tr = tf.transform.translation
        T[:3, 3] = [tr.x, tr.y, tr.z]
        K = np.array(self.msgs["info"].k, float).reshape(3, 3)
        return (image_to_array(self.msgs["rgb"])[:, :, :3], image_to_array(self.msgs["depth"]),
                image_to_array(self.msgs["labels"]), K, T)


def iou(a, b):
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / union) if union else 0.0


def save_overlay(path, rgb, results):
    import cv2
    img = rgb[:, :, ::-1].copy()
    colors = [(255, 0, 255), (0, 255, 0), (255, 128, 0), (0, 200, 255), (0, 0, 255)]
    for i, r in enumerate(results):
        c = np.array(colors[i % len(colors)])
        img[r["mask"]] = (0.45 * img[r["mask"]] + 0.55 * c).astype(np.uint8)
        x0, y0, x1, y1 = r["box"].astype(int)
        cv2.rectangle(img, (x0, y0), (x1, y1), tuple(int(v) for v in c), 1)
        label = r["name"] or "?"
        if r["iou"] is not None:
            label += f" IoU {r['iou']:.2f}"
        cv2.putText(img, label, (x0, max(y0 - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    tuple(int(v) for v in c), 1)
    cv2.imwrite(path, img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="scan_eval.png")
    args = ap.parse_args()

    registry = Registry()
    rclpy.init()
    node = FrameGrabber()
    try:
        pipeline = ScanPipeline(registry, Sam2Segmenter(), RobotFilter(node.wait_for_urdf()))
        rgb, depth, labels, K, T = node.grab()
        res = pipeline.run(rgb, depth, K, T, link_transform=node.link_transform)

        n_robot = sum(len(p) for p in pipeline.robot_filter.link_points.values())
        print(f"table plane z = {res.table_z:.4f} m, {len(res.detections)} candidate blob(s), "
              f"robot filter: {len(pipeline.robot_filter.link_points)} links, {n_robot} points")
        print("timing: " + ", ".join(f"{k} {1000 * v:.0f} ms" for k, v in res.timing.items()))
        results = []
        for i, d in enumerate(res.detections):
            spec = registry.objects.get(d.name)
            r = {"box": d.box, "mask": d.mask, "name": d.name, "iou": None}
            if spec is not None:
                r["iou"] = iou(d.mask, labels[:, :, 2] == spec.seg_label)
            sig_txt = "n/a" if d.signature is None else "h={:.1f} L={:.1f} W={:.1f} mm".format(*(1000 * d.signature))
            print(f"  blob {i}: box {d.box.astype(int).tolist()}, SAM score {d.mask_score:.2f}, {sig_txt} -> "
                  f"{d.name or 'unidentified'}"
                  + (f" (size error {100 * d.size_error:.0f}%), IoU vs ground truth {r['iou']:.3f}" if spec else ""))
            if d.T is not None:
                dt, dr = pose_error(d.T, gz_model_pose(d.name), spec.symmetry)
                yaw = np.degrees(np.arctan2(d.T[1, 0], d.T[0, 0]))
                print(f"      pose: xyz {np.round(d.T[:3, 3], 4).tolist()}, yaw {yaw:.1f} deg, "
                      f"fit {d.fitness:.2f}, rmse {1000 * d.rmse:.1f} mm, {d.n_points} pts | "
                      f"error vs ground truth: {1000 * dt:.1f} mm, {dr:.1f} deg (symmetry-aware)")
            results.append(r)
        found = res.by_name()
        for name, spec in registry.objects.items():
            if name not in found:
                gt_px = int((labels[:, :, 2] == spec.seg_label).sum())
                print(f"  MISSED {name} (ground truth has {gt_px} px)")
        save_overlay(args.out, rgb, results)
        print(f"overlay saved to {args.out}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
