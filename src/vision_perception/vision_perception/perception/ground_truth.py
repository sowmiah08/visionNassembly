"""True object poses and visible pixels straight from Gazebo. EVALUATION
ONLY: the perception pipeline must never use this.

Reads Gazebo's /world/<world>/pose/info (static models too) with the
gz-transport Python bindings. This is faster and more reliable under load
than the `gz model` command, which is kept as a fallback.
"""

import re
import subprocess
import threading
import time

import numpy as np

from vision_perception.object_registry import pose_matrix

_lock = threading.Lock()
_latest = {}        # model name -> (pose msg, arrival counter)
_counter = [0]
_node = None


def _quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _ensure_subscribed(world):
    global _node
    if _node is not None:
        return True
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
    except ImportError:
        return False

    def on_poses(msg):
        with _lock:
            _counter[0] += 1
            for p in msg.pose:
                # Keep the first entry per name: the model comes before its
                # links, and link names can repeat.
                if p.name not in _latest or _latest[p.name][1] != _counter[0]:
                    _latest[p.name] = (p, _counter[0])

    _node = Node()
    return _node.subscribe(Pose_V, f"/world/{world}/pose/info", on_poses)


def gz_model_pose(name, world="default", timeout=5.0):
    """4x4 world pose of a Gazebo model, from a message received after this call."""
    if _ensure_subscribed(world):
        with _lock:
            start = _counter[0]
        t0 = time.time()
        while time.time() - t0 < timeout:
            with _lock:
                entry = _latest.get(name)
            if entry is not None and entry[1] > start:
                p = entry[0]
                T = np.eye(4)
                T[:3, :3] = _quat_matrix(p.orientation)
                T[:3, 3] = [p.position.x, p.position.y, p.position.z]
                return T
            time.sleep(0.02)
    return _gz_model_pose_cli(name)


def _gz_model_pose_cli(name, timeout=10.0, tries=3):
    """Fallback: `gz model -m <name> -p` (slower, can time out under load)."""
    m = None
    for _ in range(tries):
        try:
            out = subprocess.run(["gz", "model", "-m", name, "-p"], capture_output=True,
                                 text=True, timeout=timeout).stdout
        except subprocess.TimeoutExpired:
            out = ""
        m = re.search(r"Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]+)\]\s*\[([^\]]+)\]", out)
        if m:
            break
    if not m:
        raise RuntimeError(f"No pose for Gazebo model '{name}' (is it spawned?)")
    return pose_matrix([float(v) for v in m.group(1).split()], [float(v) for v in m.group(2).split()])


_labels = {"frame": None, "count": 0}
_labels_node = None


def visible_pixels(label, topic="/overhead_camera/segmentation/labels_map", timeout=5.0, skip=2):
    """Pixels of `label` in the overhead ground-truth label image, or None.

    Uses a frame from after this call (plus `skip` more, so a pose change
    made just before is rendered). The label is in channel 2.
    """
    global _labels_node
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    if _labels_node is None:
        def on_image(msg):
            with _lock:
                _labels["frame"] = msg
                _labels["count"] += 1
        _labels_node = Node()
        if not _labels_node.subscribe(Image, topic, on_image):
            raise RuntimeError(f"could not subscribe to {topic}")
    with _lock:
        start = _labels["count"]
    t0 = time.time()
    while time.time() - t0 < timeout:
        with _lock:
            n, msg = _labels["count"], _labels["frame"]
        if n > start + skip:
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
            return int((img[:, :, 2] == label).sum())
        time.sleep(0.02)
    return None
