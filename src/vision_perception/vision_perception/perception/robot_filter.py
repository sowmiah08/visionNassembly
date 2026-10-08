"""Remove depth points that belong to the robot itself (arms, grippers,
gantry), using the robot's URDF collision shapes placed with live TF.
Works for any arm pose.
"""

import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory

from vision_perception.object_registry import pose_matrix

# Skip the table: plane removal handles it, and filtering it here would
# also cut off the bottom of every object on it.
SKIP_LINKS = {"world", "table_link"}


def _sample_geometry(geom, n_per_m2=40000):
    import open3d as o3d
    tag = geom.tag
    if tag == "box":
        sx, sy, sz = (float(v) for v in geom.get("size").split())
        m = o3d.geometry.TriangleMesh.create_box(sx, sy, sz)
        m.translate((-sx / 2, -sy / 2, -sz / 2))
    elif tag == "cylinder":
        m = o3d.geometry.TriangleMesh.create_cylinder(float(geom.get("radius")), float(geom.get("length")))
    elif tag == "sphere":
        m = o3d.geometry.TriangleMesh.create_sphere(float(geom.get("radius")))
    elif tag == "mesh":
        uri = geom.get("filename")
        if uri.startswith("package://"):
            pkg, rel = uri[len("package://"):].split("/", 1)
            uri = os.path.join(get_package_share_directory(pkg), rel)
        m = o3d.io.read_triangle_mesh(uri)
        if geom.get("scale"):
            s = np.array([float(v) for v in geom.get("scale").split()])
            m.vertices = o3d.utility.Vector3dVector(np.asarray(m.vertices) * s)
    else:
        return np.zeros((0, 3))
    area = max(m.get_surface_area(), 1e-6)
    n = int(min(max(area * n_per_m2, 50), 4000))
    return np.asarray(m.sample_points_uniformly(n).points)


class RobotFilter:
    def __init__(self, urdf_xml, skip_links=SKIP_LINKS):
        root = ET.fromstring(re.sub(r"<!--.*?-->", "", urdf_xml, flags=re.S))
        self.link_points = {}
        for link in root.findall("link"):
            name = link.get("name")
            if name in skip_links:
                continue
            pts = []
            for col in link.findall("collision"):
                o = col.find("origin")
                T = pose_matrix(
                    [float(v) for v in (o.get("xyz") if o is not None and o.get("xyz") else "0 0 0").split()],
                    [float(v) for v in (o.get("rpy") if o is not None and o.get("rpy") else "0 0 0").split()])
                p = _sample_geometry(col.find("geometry")[0])
                if len(p):
                    pts.append(p @ T[:3, :3].T + T[:3, 3])
            if pts:
                self.link_points[name] = np.vstack(pts)

    def robot_points(self, link_transform):
        """All robot surface points in world, given link name -> 4x4 world pose."""
        out = []
        for name, p in self.link_points.items():
            T = link_transform(name)
            if T is not None:
                out.append(p @ T[:3, :3].T + T[:3, 3])
        return np.vstack(out) if out else np.zeros((0, 3))

    def keep_mask(self, points, robot_points, radius=0.008):
        """True for scene points farther than `radius` from the robot."""
        from scipy.spatial import cKDTree
        if len(robot_points) == 0 or len(points) == 0:
            return np.ones(len(points), bool)
        d, _ = cKDTree(robot_points).query(points, distance_upper_bound=radius)
        return ~np.isfinite(d)
