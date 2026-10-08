"""One scan: depth prompts -> SAM 2 masks -> identify -> pose.

Used by both scan_eval.py (testing) and perception_node.py (the ROS service),
so both run the same code.
"""

import time
from dataclasses import dataclass, field

import numpy as np

from vision_perception.perception.depth_prompts import Workspace, find_object_clusters, mask_points
from vision_perception.perception.identify import blob_signature, identify
from vision_perception.perception.pose import estimate_pose


@dataclass
class Detection:
    name: str                   # None if the blob matched no known object
    T: np.ndarray               # 4x4 world <- object, or None
    fitness: float
    rmse: float
    n_points: int
    mask_score: float
    mask: np.ndarray            # HxW bool
    box: np.ndarray             # [x0, y0, x1, y1]
    signature: np.ndarray       # [height, long, short] metres, or None
    size_error: float           # mean relative size error vs the model, or None


@dataclass
class ScanResult:
    detections: list
    table_z: float
    timing: dict = field(default_factory=dict)

    def by_name(self):
        return {d.name: d for d in self.detections if d.name is not None}


class ScanPipeline:
    def __init__(self, registry, segmenter, robot_filter=None):
        self.registry = registry
        self.segmenter = segmenter
        self.robot_filter = robot_filter
        # Model points never change, so compute them once.
        self._models = {name: spec.model_points_normals(4000)
                        for name, spec in registry.objects.items()}

    def run(self, rgb, depth, K, T_world_cam, link_transform=None, objects=None,
            hints=None, hint_radius=0.05):
        """Find objects and their poses in one RGB-D frame.

        objects: names to look for (default: all known objects).
        hints: {name: 4x4 expected pose}. A hinted object is the blob nearest
        its hint (within hint_radius), fitted from the hint, without size
        identification. Used for a part in the gripper or seated in its target.
        """
        hints = hints or {}
        timing = {}
        t = time.time()
        keep = None
        if self.robot_filter is not None and link_transform is not None:
            robot_pts = self.robot_filter.robot_points(link_transform)
            keep = lambda p: self.robot_filter.keep_mask(p, robot_pts)
        clusters, table_z = find_object_clusters(depth, K, T_world_cam, robot_keep=keep)
        timing["prompts"] = time.time() - t

        t = time.time()
        masks, scores = self.segmenter.segment_boxes(rgb, [c.box for c in clusters])
        timing["segment"] = time.time() - t

        t = time.time()
        names = list(objects or self.registry.objects)
        blob_pts = [self._blob_points(m, c, depth, K, T_world_cam, table_z) for m, c in zip(masks, clusters)]
        sigs = [blob_signature(p, table_z) for p in blob_pts]
        # Several hinted objects can share one blob (a plug seated in its
        # socket). They are fitted in order, each removing the points it
        # explains before the next.
        hinted = {}                                   # blob index -> [spec, ...]
        for name, H in hints.items():
            dists = [np.linalg.norm(c.points[:, :2].mean(0) - H[:2, 3]) for c in clusters]
            if dists and min(dists) < hint_radius:
                hinted.setdefault(int(np.argmin(dists)), []).append(self.registry.objects[name])
        free = [i for i in range(len(clusters)) if i not in hinted]
        specs = [self.registry.objects[n] for n in names if n not in hints]
        ids = dict(zip(free, identify([sigs[i] for i in free], specs)))
        timing["identify"] = time.time() - t

        t = time.time()
        detections = []
        cam = T_world_cam[:3, 3]
        for i, (c, m, sc, p, sig) in enumerate(zip(clusters, masks, scores, blob_pts, sigs)):
            # Only points above the table, or the model's bottom face would
            # match table pixels inside the mask.
            pts = p[p[:, 2] > table_z + 0.002]
            if i in hinted:
                shared = len(hinted[i]) > 1
                for spec in hinted[i]:
                    d = Detection(spec.name, None, 0.0, float("nan"), 0, float(sc), m, c.box, sig, None)
                    if spec.static:
                        # A fixture doesn't move, so its hint is its pose. Just
                        # score it and remove its points for the next object.
                        T = hints[spec.name]
                        d.T = T
                        d.fitness, d.rmse, d.n_points = self._local_fit(pts, spec.name, T)
                        pts = self._unexplained(pts, spec.name, T)
                        detections.append(d)
                        continue
                    pose = estimate_pose(pts, spec, cam_pos=cam, model=self._models[spec.name],
                                         hint=hints[spec.name])
                    if pose is not None:
                        d.T, d.fitness, d.rmse, d.n_points = pose.T, pose.fitness, pose.rmse, pose.n_points
                        if shared:
                            # Score it only on its own points, not the whole blob.
                            d.fitness, d.rmse, d.n_points = self._local_fit(pts, spec.name, pose.T)
                        pts = self._unexplained(pts, spec.name, pose.T)
                    detections.append(d)
                continue
            spec, err = ids.get(i, (None, None))
            d = Detection(None, None, 0.0, float("nan"), 0, float(sc), m, c.box, sig, err)
            if spec is not None:
                d.name = spec.name
                # Found in the scene (no hint): it rests on the table.
                pose = estimate_pose(pts, spec, cam_pos=cam, model=self._models[spec.name],
                                     support_z=table_z)
                if pose is not None:
                    d.T, d.fitness, d.rmse, d.n_points = pose.T, pose.fitness, pose.rmse, pose.n_points
            detections.append(d)
        timing["pose"] = time.time() - t
        return ScanResult(detections, table_z, timing)

    @staticmethod
    def _blob_points(mask, cluster, depth, K, T_world_cam, table_z, ws=Workspace(), min_points=30):
        """World points under a SAM mask, kept only at heights where objects
        can be (the same band as the depth prompts).

        SAM works on the RGB image, so a mask can include something in front
        of the object (for example an occluder above it). If almost nothing is
        left in the band, use the cluster's own depth points instead.
        """
        pts = mask_points(mask, depth, K, T_world_cam)
        h = pts[:, 2] - table_z
        pts = pts[(h > ws.min_height) & (h < ws.max_height)]
        if len(pts) < min_points:
            pts = cluster.points
        return pts

    def _local_fit(self, pts, name, T, margin=0.005, radius=0.004):
        """(fitness, rmse, n) using only the scene points inside the object's
        bounding box at pose T (plus a margin)."""
        from scipy.spatial import cKDTree
        mp = self._models[name][0]
        lo, hi = mp.min(0) - margin, mp.max(0) + margin
        Ti = np.linalg.inv(T)
        local = pts @ Ti[:3, :3].T + Ti[:3, 3]
        inside = np.all((local >= lo) & (local <= hi), axis=1)
        if inside.sum() < 10:
            return 0.0, float("nan"), int(inside.sum())
        d, _ = cKDTree(mp).query(local[inside])
        ok = d < radius
        rmse = float(np.sqrt(np.mean(d[ok] ** 2))) if ok.any() else float("nan")
        return float(ok.mean()), rmse, int(inside.sum())

    def _unexplained(self, pts, name, T, radius=0.003):
        """Scene points farther than `radius` from object `name`'s model at pose T."""
        from scipy.spatial import cKDTree
        mp = self._models[name][0] @ T[:3, :3].T + T[:3, 3]
        d, _ = cKDTree(mp).query(pts, distance_upper_bound=radius)
        return pts[~np.isfinite(d)]
