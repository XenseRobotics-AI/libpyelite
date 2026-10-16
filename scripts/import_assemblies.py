"""Snapshot Elite station URDFs and derive absolute-angle native control chains.

Run with the libpyelite build environment (NumPy/Pinocchio). No hardware I/O.
The destination must not exist; regenerating a snapshot is an explicit action.
"""
import argparse
import copy
import hashlib
import json
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

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
        # Tool/adapter geometry stays in the viewer. Native arm IK has six DoFs.
        if original.get("name", "").startswith((prefix + "tool_", prefix + "adapter_")):
            continue
        if original.tag == "joint" and any(
            original.find(tag).get("link").startswith((prefix + "tool_", prefix + "adapter_"))
            for tag in ("parent", "child")
        ):
            continue
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


def import_assemblies(source, destination, identification_readme, recipe_root):
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
        record = json.loads((assembly_dir / "assembly.taccap.json").read_text())
        urdf = assembly_dir / record["entrypoint"]
        root = ET.parse(urdf).getroot()
        start_pose = json.loads((assembly_dir / "config/start_pose.taccap.json").read_text())
        if start_pose["unit"] != "radian":
            raise ValueError("Expected radians")
        # Copy only the selected assembly and its authoritative recipe/start pose.
        for path in (urdf, assembly_dir / "assembly.taccap.json",
                     assembly_dir / "config/start_pose.taccap.json",
                     assembly_dir / "README.taccap.md", assembly_dir / "COMPONENTS.taccap.md"):
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
            "joint_mapping": "q_assembly = q_controller; no baked initial offsets",
            "viewer_joint_defaults": {},
            "arms": {},
        }
        for side in ("left", "right"):
            initial = start_pose["values"][side + "_arm"]
            for joint in root.findall("joint"):
                if joint.get("name", "").startswith(side + "_tool_") and joint.get("type") != "fixed":
                    value = start_pose["values"][side + "_gripper"][0]
                    mimic = joint.find("mimic")
                    if mimic is not None:
                        value = value * float(mimic.get("multiplier", 1)) + float(mimic.get("offset", 0))
                    metadata["viewer_joint_defaults"][joint.get("name")] = value
            chain = derive_chain(root, side, [0.] * 6)
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
                "derivation": "scripts/import_assemblies.py; modern absolute-angle TacCap assemblies, strip tool subtree for six-axis native chains"}
    # Preserve exact identification source; these are aggregate tool parameters,
    # not per-finger mass/inertia or calibrated Elite TCP coordinates.
    identification_text = identification_readme.read_text()
    match = re.search(r"TacCap tool\s+tool_dynamic\s*=\s*\[([^]]+)\]", identification_text)
    if match is None:
        raise ValueError("TacCap identification array not found")
    values = [float(v.strip()) for v in match.group(1).split(",") if v.strip()]
    if len(values) != 10 or values[0] <= 0:
        raise ValueError("Invalid identification array")
    (destination / "taccap_identification_source.md").write_text(identification_text)
    payload = {"source": "lerobot-xensehand/README.md: TacCap tool",
               "source_sha256": digest(identification_readme), "mass_kg": values[0],
               "com_m_in_identification_flange": (np.array(values[1:4]) / 1000).tolist(),
               "inertia_source_order_ixx_ixy_ixz_iyy_iyz_izz": values[4:],
               "raw_tool_dynamic": values,
               "scope": "aggregate identified tool, not per-link inertia; flange convention/installation transfer needs confirmation"}
    (destination / "taccap_identification.json").write_text(json.dumps(payload, indent=2) + "\n")
    # Explicit aggregate-load approximation requested by the user. Keep every
    # source file byte-identical; only the derived viewer receives the payload.
    payload["urdf_convention"] = (
        "Equivalent rigid tool at adapter mount; COM in metres. Source inertia "
        "is retained as an effective tensor at COM in mount axes, NOT validated "
        "per-link or transferable hardware dynamics. Identification frame transfer "
        "and inertia reference-point convention require physical confirmation."
    )
    tensor = np.array([[values[4], values[5], values[6]],
                       [values[5], values[7], values[8]],
                       [values[6], values[8], values[9]]])
    eig = np.linalg.eigvalsh(tensor)
    payload["inertia_positive_definite"] = bool(np.all(eig > 0))
    payload["inertia_triangle_inequality_satisfied"] = bool(eig[-1] <= eig[0] + eig[1])
    payload["dynamics_validated"] = False
    for name in ("diagonal-07", "diagonal-08"):
        station = destination / name
        meta_path = station / "control.json"
        metadata = json.loads(meta_path.read_text())
        source_urdf = (station / metadata["assembly_urdf"]).resolve()
        root = ET.parse(source_urdf).getroot()
        for side in ("left", "right"):
            for link in root.findall("link"):
                if link.get("name", "").startswith((side + "_tool_", side + "_adapter_")):
                    for inertial in list(link.findall("inertial")):
                        link.remove(inertial)
            link = next(link for link in root.findall("link")
                        if link.get("name") == side + "_adapter_taccap_mount_link")
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "origin", xyz=" ".join(map(str, payload["com_m_in_identification_flange"])), rpy="0 0 0")
            ET.SubElement(inertial, "mass", value=str(values[0]))
            ET.SubElement(inertial, "inertia", dict(zip(("ixx", "ixy", "ixz", "iyy", "iyz", "izz"), map(str, values[4:]), strict=True)))
        # URDF relative mesh paths must follow the derived file's new location.
        for mesh in root.iter("mesh"):
            resolved = (source_urdf.parent / mesh.get("filename")).resolve()
            mesh.set("filename", os.path.relpath(resolved, station))
        derived = station / "assembly_taccap_identified.urdf"
        ET.indent(root)
        ET.ElementTree(root).write(derived, encoding="utf-8", xml_declaration=True)
        from generate_collision_profile import generate
        generate(derived, station / "collision.json")
        metadata["source_assembly_urdf"] = metadata["assembly_urdf"]
        metadata["source_assembly_sha256"] = metadata["assembly_sha256"]
        metadata["assembly_urdf"] = derived.name
        metadata["assembly_sha256"] = digest(derived)
        metadata["payload_identification"] = "../taccap_identification.json"
        meta_path.write_text(json.dumps(metadata, indent=2) + "\n")
    (destination / "taccap_identification.json").write_text(json.dumps(payload, indent=2) + "\n")
    write_simulation_profiles(destination, recipe_root)
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"Imported two assemblies, {len(copied)} source files into {destination}")


# Recipe keys the simulator needs besides every native_* key (motion limits,
# target-jump guard, collision switch, contact monitor), which are copied as-is.
SIMULATION_PROFILE_KEYS = ("type", "servoj_time", "command_stale_timeout_s", "max_lin_speed", "max_ang_speed",
                           "command_timeout_ms", "left_tool_transform", "right_tool_transform",
                           "left_start_position_rad", "right_start_position_rad")


def write_simulation_profiles(destination, recipe_root):
    """Snapshot each station's TacCap teleop recipe for the GUI's default settings."""
    import yaml
    for name in ("diagonal-07", "diagonal-08"):
        recipe = recipe_root / f"{name}-taccap.yaml"
        values = yaml.safe_load(recipe.read_text())["robot"]
        profile = {key: value for key, value in values.items()
                   if key in SIMULATION_PROFILE_KEYS or key.startswith("native_")}
        profile["source"] = {"path": f"recipes/teleop/bi_elite_cs66_rt/{recipe.name}", "sha256": digest(recipe)}
        (destination / name / "simulation_profile.json").write_text(json.dumps(profile, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, nargs="?")
    parser.add_argument("--destination", type=Path,
                        default=Path(__file__).resolve().parents[1] / "libpyelite/assets/stations")
    parser.add_argument("--identification-readme", type=Path)
    parser.add_argument("--recipe-root", type=Path, default=Path(__file__).resolve().parents[3] / "recipes/teleop/bi_elite_cs66_rt")
    parser.add_argument("--profiles-only", action="store_true",
                        help="only refresh simulation_profile.json from the recipes (after a recipe edit)")
    args = parser.parse_args()
    if args.profiles_only:
        write_simulation_profiles(args.destination, args.recipe_root)
    elif args.source is None or args.identification_readme is None:
        parser.error("source and --identification-readme are required unless --profiles-only")
    else:
        import_assemblies(args.source, args.destination, args.identification_readme, args.recipe_root)
