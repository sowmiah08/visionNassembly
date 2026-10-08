"""Object, gripper and assembly descriptions, loaded from config/*.yaml.

Everything about a specific object (3D model, symmetry, grasps, mate
frames) lives in these files, so a new object needs a YAML file, not code.
Each object's 3D model is built from its SDF, so it matches what Gazebo shows.
"""

import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def _resolve_uri(uri):
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://"):].split("/", 1)
        return os.path.join(get_package_share_directory(pkg), rel)
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose_matrix(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _yaml_pose(d):
    return pose_matrix(d.get("xyz", [0, 0, 0]), d.get("rpy", [0, 0, 0]))


def _sdf_pose(elem):
    if elem is None or not (elem.text or "").strip():
        return np.eye(4)
    v = [float(x) for x in elem.text.split()]
    return pose_matrix(v[:3], v[3:6])


@dataclass
class Symmetry:
    axis: np.ndarray
    order: int                 # 1 = no symmetry; copies used for grasps / pose search
    continuous: bool = False   # same at every angle (cylinder): its yaw can't be seen

    def rotations(self):
        """Rotation matrices of every symmetric copy, identity first."""
        a = self.axis / np.linalg.norm(self.axis)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        out = []
        for k in range(self.order):
            t = 2 * math.pi * k / self.order
            out.append(np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K)
        return out


@dataclass
class Grasp:
    name: str
    tcp_pose: np.ndarray        # 4x4, object frame
    pregrasp_offset: float
    gripper: str


@dataclass
class ObjectSpec:
    name: str
    sdf_path: str
    seg_label: int
    prompt: str
    static: bool
    symmetry: Symmetry
    grasps: list
    mate_frames: dict           # name -> 4x4, object frame
    _mesh: object = field(default=None, repr=False)

    def grasp_candidates(self):
        """Every grasp plus its symmetric copies (object frame)."""
        out = []
        for g in self.grasps:
            for k, R in enumerate(self.symmetry.rotations()):
                S = np.eye(4)
                S[:3, :3] = R
                out.append(Grasp(f"{g.name}@{k}", S @ g.tcp_pose, g.pregrasp_offset, g.gripper))
        return out

    def mesh(self):
        """Open3D TriangleMesh of the object in its own frame (from the SDF)."""
        if self._mesh is None:
            self._mesh = build_sdf_mesh(self.sdf_path)
        return self._mesh

    def model_points(self, n=4000, seed=0):
        """Points sampled evenly over the surface, for registration."""
        return self.model_points_normals(n, seed)[0]

    def model_points_normals(self, n=4000, seed=0):
        """(points, outward unit normals), both Nx3 in the object frame."""
        import open3d as o3d
        o3d.utility.random.seed(seed)
        mesh = self.mesh()
        mesh.compute_triangle_normals()
        pcd = mesh.sample_points_poisson_disk(n, use_triangle_normal=True)
        return np.asarray(pcd.points), np.asarray(pcd.normals)


@dataclass
class GripperSpec:
    name: str
    tcp_link: str
    approach_axis: np.ndarray
    joint: str
    open: float
    close: float
    touch_links: list
    held_margin: float = 0.05

    def for_side(self, side):
        f = lambda s: s.format(side=side)
        return GripperSpec(self.name, f(self.tcp_link), self.approach_axis, f(self.joint),
                           self.open, self.close, [f(t) for t in self.touch_links], self.held_margin)


@dataclass
class AssemblySpec:
    name: str
    part: str
    part_mate: str
    target: str
    target_mate: str
    approach_axis_world: np.ndarray
    pre_insert_offset: float
    speed_scale: float
    strategy: str
    xy_tol: float
    z_tol: float


def build_sdf_mesh(sdf_path):
    """Merge every visual of every link in an SDF model into one mesh."""
    import open3d as o3d
    # Remove comments first: some contain "--", which Python's XML parser rejects.
    text = re.sub(r"<!--.*?-->", "", open(sdf_path).read(), flags=re.S)
    model = ET.fromstring(text).find("model")
    merged = o3d.geometry.TriangleMesh()
    for link in model.findall("link"):
        T_link = _sdf_pose(link.find("pose"))
        for vis in link.findall("visual"):
            geom = vis.find("geometry")[0]
            if geom.tag == "box":
                sx, sy, sz = (float(v) for v in geom.find("size").text.split())
                m = o3d.geometry.TriangleMesh.create_box(sx, sy, sz)
                m.translate((-sx / 2, -sy / 2, -sz / 2))
            elif geom.tag == "cylinder":
                m = o3d.geometry.TriangleMesh.create_cylinder(
                    float(geom.find("radius").text), float(geom.find("length").text), resolution=40)
            elif geom.tag == "sphere":
                m = o3d.geometry.TriangleMesh.create_sphere(float(geom.find("radius").text), resolution=30)
            elif geom.tag == "mesh":
                m = o3d.io.read_triangle_mesh(_resolve_uri(geom.find("uri").text.strip()))
                scale = geom.find("scale")
                if scale is not None:
                    s = [float(v) for v in scale.text.split()]
                    m.vertices = o3d.utility.Vector3dVector(np.asarray(m.vertices) * np.array(s))
            else:
                continue
            m.transform(T_link @ _sdf_pose(vis.find("pose")))
            merged += m
    merged.merge_close_vertices(1e-6)
    merged.compute_vertex_normals()
    return merged


class Registry:
    """All object, gripper and assembly specs under a config directory."""

    def __init__(self, config_dir=None):
        if config_dir is None:
            config_dir = os.path.join(get_package_share_directory("vision_perception"), "config")
        self.config_dir = config_dir
        self.objects = {}
        self.grippers = {}
        self.assemblies = {}
        for fn in sorted(os.listdir(os.path.join(config_dir, "grippers"))):
            d = self._load("grippers", fn)
            self.grippers[d["name"]] = GripperSpec(
                d["name"], d["tcp_link"], np.array(d["approach_axis"], float), d["joint"],
                float(d["open"]), float(d["close"]), list(d["touch_links"]),
                float(d.get("held_margin", 0.05)))
        for fn in sorted(os.listdir(os.path.join(config_dir, "objects"))):
            d = self._load("objects", fn)
            sym = d.get("symmetry") or {"axis": [0, 0, 1], "order": 1}
            grasps = [Grasp(g["name"], _yaml_pose(g["tcp_pose"]), float(g.get("pregrasp_offset", 0.03)),
                            g.get("gripper", "so101")) for g in d.get("grasps") or []]
            for g in grasps:
                if g.gripper not in self.grippers:
                    raise ValueError(f"{fn}: grasp {g.name} uses unknown gripper '{g.gripper}'")
            self.objects[d["name"]] = ObjectSpec(
                d["name"], _resolve_uri(d["sdf"]), int(d["seg_label"]), d.get("prompt", d["name"]),
                bool(d.get("static", False)),
                Symmetry(np.array(sym["axis"], float), int(sym["order"]), bool(sym.get("continuous", False))),
                grasps, {k: _yaml_pose(v) for k, v in (d.get("mate_frames") or {}).items()})
        labels = [o.seg_label for o in self.objects.values()]
        if len(labels) != len(set(labels)):
            raise ValueError(f"seg_label values must be unique per object type: {labels}")
        for fn in sorted(os.listdir(os.path.join(config_dir, "assemblies"))):
            d = self._load("assemblies", fn)
            a = AssemblySpec(
                d["name"], d["part"]["object"], d["part"]["mate_frame"], d["target"]["object"],
                d["target"]["mate_frame"], np.array(d["approach_axis_world"], float),
                float(d["pre_insert_offset"]), float(d.get("speed_scale", 0.1)),
                d.get("strategy", "straight"), float(d["success"]["xy_tol"]), float(d["success"]["z_tol"]))
            for obj, mate in ((a.part, a.part_mate), (a.target, a.target_mate)):
                if mate not in self.objects[obj].mate_frames:
                    raise ValueError(f"{fn}: {obj} has no mate frame '{mate}'")
            self.assemblies[a.name] = a

    def _load(self, sub, fn):
        with open(os.path.join(self.config_dir, sub, fn)) as f:
            return yaml.safe_load(f)

    def by_label(self, label):
        for o in self.objects.values():
            if o.seg_label == label:
                return o
        return None

    def mated_pose(self, assembly, target_pose):
        """World pose of the part's frame when mated onto a target at `target_pose`.

        Solves part_pose @ part_mate = target_pose @ target_mate.
        """
        a = self.assemblies[assembly]
        T_pm = self.objects[a.part].mate_frames[a.part_mate]
        T_tm = self.objects[a.target].mate_frames[a.target_mate]
        return target_pose @ T_tm @ np.linalg.inv(T_pm)
