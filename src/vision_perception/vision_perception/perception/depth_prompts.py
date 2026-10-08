"""Turn a depth image into one box prompt per object on the table.

depth -> 3D points -> remove the table plane and the robot -> keep points
just above the table -> group them by footprint (x, y) -> each group's
pixel box is a SAM 2 prompt. Anything that sticks up off the table counts.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Workspace:
    """Region where objects can be (world frame, metres)."""
    x: tuple = (-0.48, 0.48)
    y: tuple = (-0.24, 0.24)
    max_height: float = 0.15        # above the table
    min_height: float = 0.004       # ignore points this close to the table


@dataclass
class Cluster:
    points: np.ndarray       # Nx3 world
    pixels: np.ndarray       # Nx2 (u, v)
    box: np.ndarray          # [x0, y0, x1, y1] pixels, padded


def depth_to_world(depth, K, T_world_cam, stride=1):
    """Back-project a metric depth image. Returns (Nx3 world points, Nx2 pixels)."""
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    ok = np.isfinite(z) & (z > 0)
    u, v, z = u[ok], v[ok], z[ok]
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    pc = np.stack([x, y, z], 1)
    pw = pc @ T_world_cam[:3, :3].T + T_world_cam[:3, 3]
    return pw, np.stack([u, v], 1)


def find_object_clusters(depth, K, T_world_cam, ws=Workspace(), robot_keep=None,
                         stride=2, eps=0.008, min_points=10, pad=4):
    """Returns (clusters, table_z).

    robot_keep: optional function(points Nx3) -> bool mask of points that
    are NOT on the robot (see RobotFilter).
    Grouping uses only (x, y): from above, the top faces of one object can
    be at different heights with nothing visible joining them.
    eps / min_points must suit the point spacing (about 3 mm at stride 2
    and 0.5 m). Change them if the stride or camera distance changes.
    Objects closer than eps merge into one blob.
    """
    import open3d as o3d

    pts, pix = depth_to_world(depth, K, T_world_cam, stride=stride)
    inside = ((pts[:, 0] > ws.x[0]) & (pts[:, 0] < ws.x[1]) &
              (pts[:, 1] > ws.y[0]) & (pts[:, 1] < ws.y[1]))
    pts, pix = pts[inside], pix[inside]

    # Fit the table plane to the lower points (mostly table).
    low = pts[:, 2] < np.percentile(pts[:, 2], 60)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts[low]))
    (a, b, c, d), _ = pcd.segment_plane(distance_threshold=0.002, ransac_n=3, num_iterations=500)
    n = np.array([a, b, c])
    if n[2] < 0:
        n, d = -n, -d
    height = (pts @ n + d) / np.linalg.norm(n)       # signed distance above the plane
    table_z = float(-d / n[2])

    keep = (height > ws.min_height) & (height < ws.max_height)
    pts, pix = pts[keep], pix[keep]
    if robot_keep is not None and len(pts):
        keep = robot_keep(pts)
        pts, pix = pts[keep], pix[keep]
    if len(pts) == 0:
        return [], table_z

    flat = pts.copy()
    flat[:, 2] = 0.0
    labels = np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(flat))
                        .cluster_dbscan(eps=eps, min_points=min_points))
    clusters = []
    h, w = depth.shape
    for k in range(labels.max() + 1):
        sel = labels == k
        p = pix[sel]
        box = np.array([max(p[:, 0].min() - pad, 0), max(p[:, 1].min() - pad, 0),
                        min(p[:, 0].max() + pad, w - 1), min(p[:, 1].max() + pad, h - 1)], float)
        clusters.append(Cluster(pts[sel], p, box))
    return clusters, table_z


def mask_points(mask, depth, K, T_world_cam):
    """World points of the depth pixels under a mask."""
    d = np.where(mask, depth, np.nan)
    pts, _ = depth_to_world(d, K, T_world_cam)
    return pts
