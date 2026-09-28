"""Assembly/control FK equivalence and packaging; never connects to a robot."""
import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from libpyelite import PinkIKModel
from libpyelite.assembly import Assembly

JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


class AssemblyTests(unittest.TestCase):
    def test_station_models_differ_only_in_name(self):
        roots = [ET.parse(Assembly(name).urdf).getroot() for name in ("diagonal-07", "diagonal-08")]
        for root in roots:
            root.set("name", "station")
        self.assertEqual(ET.tostring(roots[0]), ET.tostring(roots[1]))

    def test_packaged_assets_and_hashes(self):
        assets = Assembly("diagonal-08").root.parent
        manifest = json.loads((assets / "manifest.json").read_text())
        for path, expected in manifest["files_sha256"].items():
            self.assertEqual(hashlib.sha256((assets / "source_assets" / path).read_bytes()).hexdigest(), expected)
        for name in ("diagonal-07", "diagonal-08"):
            assembly = Assembly(name)
            self.assertEqual(hashlib.sha256(assembly.urdf.read_bytes()).hexdigest(),
                             assembly.metadata["assembly_sha256"])
            for mesh in ET.parse(assembly.urdf).getroot().iter("mesh"):
                target = (assembly.urdf.parent / mesh.get("filename")).resolve()
                self.assertTrue(target.is_relative_to(assets.resolve()))
                self.assertTrue(target.is_file(), str(target))
            for side in ("left", "right"):
                self.assertEqual(hashlib.sha256(Path(assembly.control_model_path(side)).read_bytes()).hexdigest(),
                                 assembly.metadata["arms"][side]["control_sha256"])

    def test_absolute_joint_fk_matches_full_assembly_at_many_poses(self):
        rng = np.random.default_rng(721)
        for name in ("diagonal-07", "diagonal-08"):
            assembly = Assembly(name)
            full = pin.buildModelFromUrdf(str(assembly.urdf))
            self.assertEqual(full.nq, 12)
            data = full.createData()
            for side in ("left", "right"):
                tool = np.eye(4)
                tool[:3, 3] = [0.0068, -0.0063, 0.1941] if side == "left" else [-0.008, -0.0065, 0.192]
                # Nonidentity rotation also tests multiplication order.
                tool[:3, :3] = pin.exp3(np.array([0.1, -0.2, 0.3]))
                control = PinkIKModel(urdf=assembly.control_model_path(side), tool=tool)
                q0 = assembly.initial_pose(side)
                indices = [full.joints[full.getJointId(side + "_" + j)].idx_q for j in JOINTS]
                np.testing.assert_allclose(full.lowerPositionLimit[indices] + q0, control.lower_limits, atol=1e-12)
                np.testing.assert_allclose(full.upperPositionLimit[indices] + q0, control.upper_limits, atol=1e-12)
                np.testing.assert_allclose(full.velocityLimit[indices], control.velocity_limits, atol=1e-12)
                for absolute_q in [q0, np.zeros(6)] + list(rng.uniform(control.lower_limits, control.upper_limits, (20, 6))):
                    relative_q = np.zeros(12)
                    relative_q[indices] = absolute_q - q0
                    pin.framesForwardKinematics(full, data, relative_q)
                    expected = data.oMf[full.getFrameId(side + "_tool0")].homogeneous @ tool
                    actual = assembly.world_from_base(side) @ control.fk(absolute_q)
                    np.testing.assert_allclose(actual, expected, atol=1e-10)

    def test_native_ik_with_assembly_control_chains(self):
        for name in ("diagonal-07", "diagonal-08"):
            assembly = Assembly(name)
            for side in ("left", "right"):
                model = PinkIKModel(urdf=assembly.control_model_path(side))
                q = assembly.initial_pose(side)
                target = model.fk(q)
                target[0, 3] += 0.002
                solved, diagnostics = model.solve(target, q)
                self.assertTrue(diagnostics.success)
                np.testing.assert_allclose(model.fk(solved)[:3, 3], target[:3, 3], atol=1e-4)


if __name__ == "__main__":
    unittest.main()
