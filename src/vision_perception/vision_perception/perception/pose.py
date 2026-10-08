"""6-DoF object pose: fit the object's 3D model (built from its SDF) to the
depth points under its mask, with ICP.

The camera sees only part of an object, so the scene points are matched
only to model points that face the camera. Otherwise a visible top face can
match the model's hidden bottom face and give a wrong pose that still looks
like a good fit.

ICP starts from several orientations (every `yaw_step` within one symmetry
period) and the best fit wins.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class PoseResult:
    T: np.ndarray            # 4x4 world <- object
    fitness: float           # fraction of scene points with a model point within max_dist
    rmse: float              # metres, over those inliers
    n_points: int


def _rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _base_orientations(upright_only):
    if upright_only:
        return [np.eye(3)]
    rx = lambda t: np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    ry = lambda t: np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    return [np.eye(3), rx(math.pi), rx(math.pi / 2), rx(-math.pi / 2), ry(math.pi / 2), ry(-math.pi / 2)]


def _visible(model_points, model_normals, T_obj, cam_pos):
    """Indices of model points whose outward normal faces the camera at pose T_obj."""
    pw = model_points @ T_obj[:3, :3].T + T_obj[:3, 3]
    nw = model_normals @ T_obj[:3, :3].T
    return np.nonzero(np.einsum("ij,ij->i", nw, np.asarray(cam_pos, float) - pw) > 0)[0]


def estimate_pose(scene_points, spec, cam_pos, max_dist=0.004, yaw_step_deg=15.0,
                  upright_only=True, model=None, voxel=0.002, hint=None,
                  support_z=None, support_tol=0.005, max_tilt_deg=5.0):
    """Best pose of `spec` for `scene_points`, or None if there are too few points.

    model: optional (points, normals) from spec.model_points_normals().
    hint: optional 4x4 expected pose. Adds starting guesses at the hint
    (yaw +-30 deg) on top of the normal search.
    support_z: optional table height. The object must then rest on it:
    guesses start there, and fits that float or tilt are rejected. With most
    of a part hidden, a few points can fit a wrong pose well; this stops
    that. Don't use it for a part in the hand or seated on another part.
    """
    import open3d as o3d
    reg = o3d.pipelines.registration

    if len(scene_points) < 30:
        return None
    scene = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scene_points))
    scene = scene.voxel_down_sample(voxel)
    # Returns the filtered cloud; it does not filter in place.
    scene, _ = scene.remove_statistical_outlier(nb_neighbors=16, std_ratio=2.5)
    sc = np.asarray(scene.points)

    model_points, model_normals = model if model is not None else spec.model_points_normals(4000)

    def model_cloud(idx):
        m = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(model_points[idx]))
        m.normals = o3d.utility.Vector3dVector(model_normals[idx])
        return m

    period = 2 * math.pi / max(spec.symmetry.order, 1)
    yaws = np.arange(0.0, period - 1e-9, math.radians(yaw_step_deg))
    best = None
    centroid = sc.mean(0)
    inits = []
    for R0 in _base_orientations(upright_only):
        for yaw in yaws:
            R = _rot_z(yaw) @ R0
            # Start with the model centred on the scene points.
            T_obj = np.eye(4)
            T_obj[:3, :3] = R
            T_obj[:3, 3] = centroid - (R @ model_points.T).T.mean(0)
            # Centre on the part of the model the camera can see. For a tall
            # thin object the camera sees mostly the top, so the full model's
            # centre would start too high.
            vis = _visible(model_points, model_normals, T_obj, cam_pos)
            if len(vis) > 10:
                T_obj[:3, 3] = centroid - (R @ model_points[vis].T).T.mean(0)
            if support_z is not None:
                # Resting on the support: bottom of the model on the plane.
                T_obj[2, 3] = support_z - (R @ model_points.T)[2].min()
            inits.append(T_obj)
    if hint is not None:
        for dyaw in np.radians(np.arange(-30, 31, 10)):
            T_obj = hint.copy()
            T_obj[:3, :3] = _rot_z(dyaw) @ hint[:3, :3]
            inits.append(T_obj)
    for T_obj in inits:
        # ICP estimates scene -> model, i.e. the inverse of the object pose.
        init = np.linalg.inv(T_obj)
        for d in (3 * max_dist, max_dist):
            visible = model_cloud(_visible(model_points, model_normals, np.linalg.inv(init), cam_pos))
            res = reg.registration_icp(
                scene, visible, d, init, reg.TransformationEstimationPointToPlane(),
                reg.ICPConvergenceCriteria(max_iteration=40))
            init = res.transformation
        cand = PoseResult(np.linalg.inv(res.transformation), res.fitness, res.inlier_rmse, len(sc))
        if support_z is not None:
            R = cand.T[:3, :3]
            bottom = (model_points @ R.T + cand.T[:3, 3])[:, 2].min()
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
            if abs(bottom - support_z) > support_tol or tilt > max_tilt_deg:
                continue
        key = (cand.fitness, -cand.rmse)
        if best is None or key > (best.fitness, -best.rmse):
            best = cand
    return best


def pose_error(T_est, T_gt, symmetry):
    """(translation error m, rotation error deg), minimised over symmetric copies."""
    dt = float(np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3]))
    if symmetry.continuous:
        # Only the axis direction is observable: angle between the axes.
        a = T_est[:3, :3] @ symmetry.axis
        b = T_gt[:3, :3] @ symmetry.axis
        cos = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
        return dt, math.degrees(math.acos(max(-1.0, min(1.0, cos))))
    best = 180.0
    for S in symmetry.rotations():
        R = T_gt[:3, :3] @ S
        cos = (np.trace(R.T @ T_est[:3, :3]) - 1) / 2
        best = min(best, math.degrees(math.acos(max(-1.0, min(1.0, cos)))))
    return dt, best
