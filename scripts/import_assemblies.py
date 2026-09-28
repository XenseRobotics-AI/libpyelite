"""Snapshot Elite station URDFs and derive absolute-angle native control chains.

Run with the libpyelite build environment (NumPy/Pinocchio). No hardware I/O.
The destination must not exist; regenerating a snapshot is an explicit action.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def origin_matrix(joint):
    origin = joint.find("origin")
    attrs = {} if origin is None else origin.attrib
    matrix = np.eye(4)
    matrix[:3, :3] = pin.rpy.rpyToMatrix(np.fromstring(attrs.get("rpy", "0 0 0"), sep=" "))
    matrix[:3, 3] = np.fromstring(attrs.get("xyz", "0 0 0"), sep=" ")
    return matrix


def fixed_transform(root, child):
    parents = {j.find("child").get("link"): j for j in root.findall("joint")}
    matrix = np.eye(4)
    visited = set()
    while child in parents:
        if child in visited:
            raise ValueError("Cycle in assembly URDF")
        visited.add(child)
        joint = parents[child]
        if joint.get("type") != "fixed":
            raise ValueError(f"Base path contains moving joint: {joint.get('name')}")
        matrix = origin_matrix(joint) @ matrix
        child = joint.find("parent").get("link")
    if child != "world":
        raise ValueError(f"Unexpected root {child}")
    return matrix


def derive_chain(root, side, initial):
    prefix = side + "_"
    chain = ET.Element("robot", name=f"cs66_{side}_absolute")
    descendants = {prefix + "world"}
    while True:
        expanded = descendants | {j.find("child").get("link") for j in root.findall("joint")
                                  if j.find("parent").get("link") in descendants}
        if expanded == descendants:
            break
        descendants = expanded
    for original in root:
        if original.tag not in ("joint", "link") or not original.get("name", "").startswith(prefix):
            continue
        if original.tag == "link" and original.get("name") not in descendants:
            continue
        if original.tag == "joint" and not all(
            original.find(tag).get("link") in descendants for tag in ("parent", "child")
        ):
            continue  # assembly attachment is represented by world_from_base
        node = copy.deepcopy(original)
        node.set("name", node.get("name")[len(prefix):])
        if node.tag == "link":
            for tag in ("visual", "collision"):
                for geometry in list(node.findall(tag)):
                    node.remove(geometry)  # kinematics-only chain, full meshes stay in assembly
        else:
            for tag in ("parent", "child"):
                element = node.find(tag)
                element.set("link", element.get("link")[len(prefix):])
        chain.append(node)
    moving = [j for j in chain.findall("joint") if j.get("type") != "fixed"]
    if tuple(j.get("name") for j in moving) != JOINTS or len(initial) != 6:
        raise ValueError("Unexpected CS66 joint order or initial pose")
    for joint, offset in zip(moving, initial, strict=True):
        if joint.get("type") != "revolute" or any(joint.find(tag) is not None for tag in ("mimic", "calibration")):
            raise ValueError("Expected independent bounded revolute joints")
        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        axis /= np.linalg.norm(axis)
        original_rotation = origin_matrix(joint)[:3, :3] @ pin.exp3(-float(offset) * axis)
        joint.find("origin").set("rpy", " ".join(format(v, ".17g") for v in pin.rpy.matrixToRpy(original_rotation)))
        for tag, keys in (("limit", ("lower", "upper")),
                          ("safety_controller", ("soft_lower_limit", "soft_upper_limit"))):
            element = joint.find(tag)
            if element is not None:
                for key in keys:
                    if key in element.attrib:
                        element.set(key, format(float(element.get(key)) + offset, ".17g"))
    # The driver speaks the controller's base frame, not the assembly world.
    np.testing.assert_allclose(fixed_transform(chain, "base"), np.eye(4), atol=1e-12)
    return chain


def import_assemblies(source, destination):
    source = source.resolve()
    if destination.exists():
        raise FileExistsError(f"Refusing to overwrite {destination}")
    destination.mkdir(parents=True)
    copied = {}

    def snapshot(path):
        path = path.resolve()
        relative = path.relative_to(source / "assets")
        target = destination / "source_assets" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        copied[str(relative)] = digest(path)
        return target

    for name in ("diagonal-07", "diagonal-08"):
        assembly_dir = source / "assets" / "assemblies" / f"{name}-elite-cs66"
        record = json.loads((assembly_dir / "assembly.json").read_text())
        urdf = assembly_dir / record["entrypoint"]
        root = ET.parse(urdf).getroot()
        if record["initial_pose"]["unit"] != "radian":
            raise ValueError("Expected radians")
        for path in assembly_dir.rglob("*"):
            if path.is_file():
                snapshot(path)
        # Snapshot all referenced meshes, and provenance/licenses of their assets.
        assets = set()
        for mesh in root.iter("mesh"):
            path = (urdf.parent / mesh.get("filename")).resolve()
            if path.suffix.lower() == ".obj":
                raise ValueError("OBJ dependency import requires explicit MTL handling")
            if path.suffix.lower() == ".dae" and list(ET.parse(path).getroot().iter("{http://www.collada.org/2005/11/COLLADASchema}image")):
                raise ValueError("DAE textures require explicit dependency handling")
            snapshot(path)
            for parent in path.parents:
                if (parent / "asset.json").is_file():
                    assets.add(parent)
                    break
            else:
                raise ValueError(f"Missing source asset metadata: {path}")
        for asset in assets:
            for path in asset.rglob("*"):
                if path.is_file() and (path.suffix in (".json", ".md") or "provenance" in path.parts or "LICENSE" in str(path.relative_to(asset))):
                    snapshot(path)
            # Validate the assembly's recorded component input against its source URDF.
            asset_record = json.loads((asset / "asset.json").read_text())
            for instance, spec in record["instances"].items():
                if spec["asset_id"] == asset_record["id"]:
                    entry = asset_record["entrypoints"][spec["variant"]]
                    if isinstance(entry, dict):
                        entry = entry["urdf"]
                    source_urdf = asset / entry
                    if digest(source_urdf) != record["inputs"][instance]:
                        raise ValueError(f"Assembly input hash drift: {instance}")
                    snapshot(source_urdf)
        station = destination / name
        station.mkdir()
        metadata = {
            "assembly_id": record["id"],
            "assembly_urdf": f"../source_assets/assemblies/{name}-elite-cs66/{record['entrypoint']}",
            "assembly_sha256": digest(urdf),
            "joint_mapping": "q_assembly = q_controller - initial_pose; control URDF uses q_controller",
            "arms": {},
        }
        for side in ("left", "right"):
            initial = record["initial_pose"]["values"][side + "_arm"]
            chain = derive_chain(root, side, initial)
            chain_path = station / f"{side}_control.urdf"
            ET.indent(chain)
            ET.ElementTree(chain).write(chain_path, encoding="utf-8", xml_declaration=True)
            metadata["arms"][side] = {
                "control_urdf": chain_path.name,
                "control_sha256": digest(chain_path),
                "initial_pose": initial,
                "world_from_base": fixed_transform(root, side + "_base").tolist(),
            }
        (station / "control.json").write_text(json.dumps(metadata, indent=2) + "\n")
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    manifest = {"source": "xense-description", "revision": revision,
                "snapshot": "working tree; file hashes are authoritative", "files_sha256": copied,
                "derivation": "scripts/import_assemblies.py; undo baked initial rotations/limits, strip arm prefix and visual/collision nodes"}
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Imported two assemblies, {len(copied)} source files into {destination}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--destination", type=Path,
                        default=Path(__file__).resolve().parents[1] / "libpyelite/assets/stations")
    args = parser.parse_args()
    import_assemblies(args.source, args.destination)
