"""Decide which known object each segmented blob is, from its 3D size:
height above the table, and the long and short sides of its footprint.

Assumes objects stand upright as modelled.
"""

import numpy as np


def model_signature(spec):
    v = np.asarray(spec.mesh().vertices)
    dx, dy = np.ptp(v[:, 0]), np.ptp(v[:, 1])
    return np.array([np.ptp(v[:, 2]), max(dx, dy), min(dx, dy)])


def blob_signature(points, table_z):
    import cv2
    # Only points above the table: a mask often includes a little table.
    points = points[points[:, 2] > table_z + 0.004]
    if len(points) < 10:
        return None
    height = np.percentile(points[:, 2], 98) - table_z
    rect = cv2.minAreaRect(points[:, :2].astype(np.float32))
    w, h = rect[1]
    return np.array([height, max(w, h), min(w, h)])


def identify(signatures, specs, max_rel_error=0.35):
    """Assign blobs to objects, one blob per object (greedy, best first).

    signatures: list of blob signatures (or None). specs: ObjectSpecs.
    Returns list of (spec or None, relative error) per blob.
    """
    models = [(s, model_signature(s)) for s in specs]
    pairs = []
    for i, sig in enumerate(signatures):
        if sig is None:
            continue
        for s, m in models:
            err = float(np.mean(np.abs(sig - m) / m))
            pairs.append((err, i, s))
    pairs.sort(key=lambda t: t[0])
    out = [(None, None)] * len(signatures)
    used_blobs, used_objs = set(), set()
    for err, i, s in pairs:
        if err > max_rel_error or i in used_blobs or s.name in used_objs:
            continue
        out[i] = (s, err)
        used_blobs.add(i)
        used_objs.add(s.name)
    return out
