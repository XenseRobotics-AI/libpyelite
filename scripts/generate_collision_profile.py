"""Fit conservative primitives offline; runtime reads JSON, never rewrites URDF.

Binary STL triangles are enclosed in capsules/cylinders/boxes. Furniture is
partitioned by triangle centroids; each complete triangle remains in its box.
"""

import argparse
import hashlib
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin


def triangles(path):
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL: {path}")
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(raw, dtype=dtype, offset=84)["vertices"].astype(float)


def boxes(tris, depth=0):
    points = tris.reshape(-1, 3)
    lo, hi = points.min(0), points.max(0)
    if depth >= 6 or max(hi - lo) < 0.12 or len(tris) < 24:
        return [(lo, hi)]
    centers = tris.mean(1)
    axis = np.argmax(np.ptp(centers, axis=0))
    split = np.median(centers[:, axis])
    mask = centers[:, axis] < split
    if not mask.any() or mask.all():
        return [(lo, hi)]
    return boxes(tris[mask], depth + 1) + boxes(tris[~mask], depth + 1)


FINGER_SLAB_M = 0.01
FINGER_SPLIT_GAIN = 0.85


def clipped_bounds(tris, lo, hi):
    """Exact bounds of the mesh surface inside the box [lo, hi], or None if empty."""
    vmin, vmax = tris.min(1), tris.max(1)
    touching = np.all((vmax >= lo) & (vmin <= hi), axis=1)
    inside = touching & np.all((vmin >= lo) & (vmax <= hi), axis=1)
    points = [tris[inside].reshape(-1, 3)]
    for tri in tris[touching & ~inside]:
        polygon = list(tri)
        for axis in range(3):
            for bound, sign in ((lo[axis], 1.0), (hi[axis], -1.0)):
                clipped = []
                for k, p in enumerate(polygon):
                    q = polygon[(k + 1) % len(polygon)]
                    dp, dq = sign * (p[axis] - bound), sign * (q[axis] - bound)
                    if dp >= 0:
                        clipped.append(p)
                    if dp * dq < 0:
                        clipped.append(p + (q - p) * (dp / (dp - dq)))
                polygon = clipped
                if not polygon:
                    break
            if not polygon:
                break
        points.append(np.asarray(polygon).reshape(-1, 3))
    points = np.concatenate(points)
    return (points.min(0), points.max(0)) if len(points) else None


def slab_boxes(tris, axis=2, thickness=FINGER_SLAB_M, candidates=7):
    """Stacked boxes that follow a tapered part's outline instead of its bounding box.

    One bounding box of a TacCap finger sticks out at its corners: with the
    gripper tilted the box reached 25-32 mm below the real fingertip, so the arm
    stopped 2-3 cm above the table. Cut the mesh into slabs along its long axis,
    take each slab's exact clipped bounds (a stepped trapezoid in profile), and
    split a slab once across when that saves enough volume (the finger's
    cross-section is an L, not a rectangle). Every surface point stays enclosed.
    """
    points = tris.reshape(-1, 3)
    lo, hi = points.min(0), points.max(0)
    edges = np.linspace(lo[axis], hi[axis], max(1, int(np.ceil((hi[axis] - lo[axis]) / thickness))) + 1)
    volume = lambda box: np.prod(box[1] - box[0] + 0.004)
    out = []
    for a, b in zip(edges[:-1], edges[1:]):
        slab_lo, slab_hi = lo.copy(), hi.copy()
        slab_lo[axis], slab_hi[axis] = a, b
        slab = clipped_bounds(tris, slab_lo, slab_hi)
        if slab is None:
            continue
        best, best_volume = [slab], volume(slab)
        for across in (i for i in range(3) if i != axis):
            for split in np.linspace(slab[0][across], slab[1][across], candidates + 2)[1:-1]:
                first_hi, second_lo = slab[1].copy(), slab[0].copy()
                first_hi[across], second_lo[across] = split, split
                parts = [p for p in (clipped_bounds(tris, slab[0], first_hi), clipped_bounds(tris, second_lo, slab[1])) if p]
                total = sum(volume(p) for p in parts)
                if total < best_volume * FINGER_SPLIT_GAIN:
                    best, best_volume = parts, total
        out += best
    return out


SEPARATION_CLEARANCE_M = 0.005
SEPARATION_MAX_JOINTS = 2


def separated_link_pairs(urdf, frames):
    """Same-arm link pairs whose source meshes provably never come within 5 mm.

    For a pair related by at most two revolute joints, branch-and-bound over the
    full joint range: a cell is proven when the mesh distance at its center,
    minus a Lipschitz bound on how far any moving vertex can travel inside the
    cell, still exceeds the clearance. Every cell must be proven; there is no
    sampling. Fixed-joint offsets in between are part of the URDF placements.
    """
    import coal

    model = pin.buildModelFromUrdf(str(urdf))
    geometry = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION, package_dirs=[str(urdf.parent)])
    data, geometry_data = model.createData(), geometry.createData()
    request = coal.DistanceRequest()
    meshes = {}
    for index, item in enumerate(geometry.geometryObjects):
        meshes.setdefault(model.frames[item.parentFrame].name, []).append(index)

    def chain(upper, lower):
        joints = []
        while lower not in (upper, 0):
            joints.append(lower)
            lower = model.parents[lower]
        return joints[::-1] if lower == upper else None

    def distance(q, first, second):
        pin.updateGeometryPlacements(model, data, geometry, geometry_data, q)
        best = np.inf
        for i in meshes[first]:
            for j in meshes[second]:
                a, b = geometry_data.oMg[i], geometry_data.oMg[j]
                best = min(
                    best,
                    coal.distance(
                        geometry.geometryObjects[i].geometry,
                        coal.Transform3s(a.rotation, a.translation),
                        geometry.geometryObjects[j].geometry,
                        coal.Transform3s(b.rotation, b.translation),
                        request,
                        coal.DistanceResult(),
                    ),
                )
        return best

    proven = []
    for a in frames:
        for b in frames:
            if a >= b or a not in meshes or b not in meshes or a.split("_")[0] != b.split("_")[0]:
                continue
            ja, jb = (model.frames[model.getFrameId(f)].parentJoint for f in (a, b))
            path, moving = chain(ja, jb), b
            if path is None:
                path, moving = chain(jb, ja), a
            if not path or len(path) > SEPARATION_MAX_JOINTS:
                continue
            if any(model.joints[j].nq != 1 or not model.joints[j].shortname().startswith("JointModelR") for j in path):
                continue
            # Radius of the moving mesh about each path joint, bounded through the
            # fixed joint offsets that follow it on the chain.
            last = model.frames[model.getFrameId(moving)].parentJoint
            reach = max(
                float(
                    np.linalg.norm(
                        geometry.geometryObjects[i].geometry.vertices() @ geometry.geometryObjects[i].placement.rotation.T
                        + geometry.geometryObjects[i].placement.translation,
                        axis=1,
                    ).max()
                )
                for i in meshes[moving]
            )
            radius = []
            for k, joint in enumerate(path):
                offsets = sum(np.linalg.norm(model.jointPlacements[j].translation) for j in path[k + 1 :])
                radius.append(offsets + reach)
            radius = np.array(radius)
            assert path[-1] == last
            index = [model.joints[j].idx_q for j in path]
            lower = np.maximum([model.lowerPositionLimit[i] for i in index], -np.pi)
            upper = np.minimum([model.upperPositionLimit[i] for i in index], np.pi)
            q = pin.neutral(model)
            cells = [((lower + upper) / 2, (upper - lower) / 2)]
            ok = True
            while cells and ok:
                center, half = cells.pop()
                q[index] = center
                if distance(q, a, b) - radius @ half > SEPARATION_CLEARANCE_M:
                    continue
                if half.max() < np.radians(0.05):
                    ok = False
                    break
                axis = int(np.argmax(radius * half))
                step = np.zeros_like(half)
                step[axis] = half[axis] / 2
                child = half.copy()
                child[axis] /= 2
                cells += [(center - step, child), (center + step, child)]
            if ok:
                proven.append([a, b])
    return proven


def generate(urdf, output):
    tree = ET.parse(urdf).getroot()
    result = {
        "schema": 1,
        "margin_m": 0.01,
        "self_margin_m": 0.0005,
        # Mink's default band; still > margin + buffer + two arms' closing per 4 ms step.
        "detection_distance_m": 0.05,
        "gain": 0.2,
        "qp_buffer_m": 0.001,
        "padding_m": 0.002,
        "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "shapes": [],
        "allowed_link_pairs": [],
        "sources": {},
    }
    parent = {j.find("child").get("link"): j.find("parent").get("link") for j in tree.findall("joint")}
    fixed = {j.find("child").get("link") for j in tree.findall("joint") if j.get("type") == "fixed"}

    def rigid(link):
        while link in fixed:
            link = parent[link]
        return link

    for link in tree.findall("link"):
        name = link.get("name")
        env = name in ("elite_cs66_a1_stand_base", "worktable_120cm_base")
        elements = link.findall("visual" if env else "collision")
        for element in elements:
            mesh = element.find("geometry/mesh")
            if mesh is None:
                continue
            path = (urdf.parent / mesh.get("filename")).resolve()
            tris = triangles(path) * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            origin = element.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            rotation = pin.rpy.rpyToMatrix(rpy)
            tris = tris @ rotation.T + xyz
            result["sources"][str(path.relative_to(urdf.parent.parent.resolve()))] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
            points = tris.reshape(-1, 3)
            lo, hi = points.min(0), points.max(0)
            shapes = []
            if env or "finger_link" in name or "wrist_" in name or "shoulder_link" in name:
                for low, high in slab_boxes(tris) if "finger_link" in name else boxes(tris):
                    shapes.append(
                        {
                            "type": "box",
                            "size": (high - low + 0.004).tolist(),
                            "xyz": ((high + low) / 2).tolist(),
                            "rpy": [0, 0, 0],
                        }
                    )
            elif any(x in name for x in ("upperarm", "forearm", "base_link_inertia")):
                axis = 2 if "base_link_inertia" in name else int(np.argmax(hi - lo))
                center = (hi + lo) / 2
                radial = np.delete(points - center, axis, axis=1)
                radius = float(np.linalg.norm(radial, axis=1).max() + 0.002)
                angles = [0, np.pi / 2, 0] if axis == 0 else [-np.pi / 2, 0, 0] if axis == 1 else [0, 0, 0]
                length = float(hi[axis] - lo[axis] + 0.004)
                if any(x in name for x in ("upperarm", "forearm")):
                    # Tight axial capsule: sphere tips, not cylinder ends, cover
                    # the endpoint vertices. All triangle vertices are enclosed.
                    z = points[:, axis]
                    cap = np.sqrt(np.maximum(0, radius**2 - np.sum(radial**2, axis=1)))
                    low, high = float(np.min(z + cap)), float(np.max(z - cap))
                    center[axis] = (low + high) / 2
                    length = max(0.0001, high - low)
                shapes.append(
                    {
                        "type": "capsule" if any(x in name for x in ("upperarm", "forearm")) else "cylinder",
                        "radius": radius,
                        "length": length,
                        "xyz": center.tolist(),
                        "rpy": angles,
                    }
                )
            else:
                shapes.append(
                    {
                        "type": "box",
                        "size": (hi - lo + 0.004).tolist(),
                        "xyz": ((hi + lo) / 2).tolist(),
                        "rpy": [0, 0, 0],
                    }
                )
            for shape in shapes:
                shape.update(
                    name=f"{name}/{len(result['shapes'])}",
                    frame=name,
                    group="environment" if env else name.split("_")[0],
                )
                result["shapes"].append(shape)
    # Only mechanically connected rigid groups are exempted, not random samples.
    names = sorted({s["frame"] for s in result["shapes"]})
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            ra, rb = rigid(a), rigid(b)
            adjacent = (
                ra == rb or (ra in parent and rigid(parent[ra]) == rb) or (rb in parent and rigid(parent[rb]) == ra)
            )
            # Fixed environment shares world with robot bases. Do not disable
            # robot/environment indiscriminately: only mounted base/shoulder vs stand.
            env_pair = any(x in (a, b) for x in ("elite_cs66_a1_stand_base", "worktable_120cm_base"))
            if a.startswith("left_") and b.startswith("right_"):
                adjacent = False
            if env_pair:
                adjacent = "elite_cs66_a1_stand_base" in (a, b) and any(
                    "base_link_inertia" in x or "shoulder_link" in x for x in (a, b)
                )
            if adjacent:
                result["allowed_link_pairs"].append([a, b])
    # TacCap mechanically coupled opposing fingers include meshing/contacting
    # root geometry in the source STL. Only this internal pair is allowed;
    # each finger still collides with both arms, the other gripper and furniture.
    for side in ("left", "right"):
        result["allowed_link_pairs"].append(
            [f"{side}_tool_taccap_left_finger_link", f"{side}_tool_taccap_right_finger_link"]
        )
    # Conservative primitives of short-chain pairs can overlap where the real
    # meshes never come close (e.g. upperarm capsule vs base cylinder at 0.9 mm
    # while the meshes stay >= 16 mm apart over the full J1/J2 range). Such a
    # permanently active barrier row only throttles nominal motion.
    allowed = {frozenset(p) for p in result["allowed_link_pairs"]}
    result["kinematically_separated_pairs"] = [
        p for p in separated_link_pairs(urdf, names) if frozenset(p) not in allowed
    ]
    result["allowed_link_pairs"] += result["kinematically_separated_pairs"]
    result["allowed_pair_policy"] = (
        "Adjacent rigid groups; mounted base/shoulder against stand; TacCap coupled finger roots (source meshes contact at default opening); "
        f"same-arm pairs within {SEPARATION_MAX_JOINTS} revolute joints whose source meshes are proven >= "
        f"{SEPARATION_CLEARANCE_M * 1000:g} mm apart over the full joint range by branch-and-bound. No sample-based disabling."
    )
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    generate(args.urdf, args.output)
