"""Hardware-free CS66 branch and mounted-model proof using native Pinocchio.

Run directly with the project's Python environment; pytest is not required.
"""

import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

try:
    import numpy as np
    import pinocchio as pin
except ImportError as exc:
    raise unittest.SkipTest("CS66 geometry proof requires numpy and pinocchio") from exc


ASSETS = Path(__file__).resolve().parents[1] / "libpyelite" / "assets"
JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
CONTROL_PATHS = [
    ASSETS / "cs66/urdf/cs66.urdf",
    *sorted((ASSETS / "stations").glob("diagonal-0[78]/*_control.urdf")),
]


def joint_records(path):
    records = []
    for joint in ET.parse(path).getroot().findall("joint"):
        if joint.attrib["type"] == "fixed":
            continue
        origin = joint.find("origin")
        records.append(
            (
                joint.attrib["name"],
                joint.attrib["type"],
                np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" "),
                np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" "),
                np.fromstring(joint.find("axis").attrib["xyz"], sep=" "),
            )
        )
    return records


def coefficients(path):
    joints = joint_records(path)
    alpha = joints[1][3][0]
    beta = joints[4][3][0]
    upper = -joints[2][2][0]
    forearm = -joints[3][2][0]
    d5 = -joints[4][2][1]
    d6 = joints[5][2][1]
    return upper, forearm, d5 - d6 * math.cos(beta), alpha, beta


def shoulder(q, geometry):
    upper, forearm, effective, _, _ = geometry
    return (
        -upper * math.cos(q[1])
        - forearm * math.cos(q[1] + q[2])
        + effective * math.sin(q[1] + q[2] + q[3])
    )


class CS66GeometryProof(unittest.TestCase):
    def test_model_signature(self):
        canonical = joint_records(CONTROL_PATHS[0])
        self.assertEqual(len(CONTROL_PATHS), 5)
        expected_xyz = [
            [0, 0, 0.1625], [0, 0, 0], [-0.427, 0, 0],
            [-0.3905, 0, 0.1475], [0, -0.0965, 0], [0, 0.095, 0],
        ]
        expected_rpy = [
            [0, 0, 0], [math.pi / 2, 0, 0], [0, 0, 0],
            [0, 0, 0], [math.pi / 2, 0, 0], [-math.pi / 2, 0, 0],
        ]
        for path in CONTROL_PATHS:
            records = joint_records(path)
            self.assertEqual(len(records), 6)
            for index, record in enumerate(records):
                name, kind, xyz, rpy, axis = record
                self.assertEqual(name, JOINT_NAMES[index])
                self.assertEqual(kind, "revolute")
                np.testing.assert_array_equal(axis, [0, 0, 1])
                np.testing.assert_allclose(xyz, expected_xyz[index], atol=1e-15, rtol=0)
                np.testing.assert_allclose(rpy, expected_rpy[index], atol=2.1e-10, rtol=0)
                np.testing.assert_allclose(rpy, canonical[index][3], atol=5e-16, rtol=0)

    def test_exact_determinant_factorization(self):
        rng = np.random.default_rng(230)
        max_error = 0.0
        for path in CONTROL_PATHS:
            geometry = coefficients(path)
            upper, forearm, _, alpha, beta = geometry
            model = pin.buildModelFromUrdf(str(path))
            data = model.createData()
            frame = model.getFrameId("tool0")
            for _ in range(2000):
                q = rng.uniform(-math.pi, math.pi, 6)
                jacobian = pin.computeFrameJacobian(
                    model, data, q, frame, pin.LOCAL_WORLD_ALIGNED
                )
                analytic = (
                    upper * forearm * math.sin(alpha) * math.sin(beta) ** 2
                    * math.sin(q[2]) * math.sin(q[4]) * shoulder(q, geometry)
                )
                error = abs(np.linalg.det(jacobian) - analytic)
                max_error = max(max_error, error)
                self.assertLess(error, 2e-15, str(path))
        print(f"10,000 Jacobian determinants: maximum absolute error {max_error:.3g}")

    def test_each_singular_surface_and_sign(self):
        for path in CONTROL_PATHS:
            geometry = coefficients(path)
            upper, forearm, effective, _, _ = geometry
            model = pin.buildModelFromUrdf(str(path))
            data = model.createData()
            frame = model.getFrameId("tool0")

            def determinant(q):
                return np.linalg.det(
                    pin.computeFrameJacobian(model, data, q, frame, pin.LOCAL_WORLD_ALIGNED)
                )

            for joint in (2, 4):
                for root in (0.0, math.pi, -math.pi):
                    q = np.array([0.31, -0.5, 1.1, -0.45, 0.9, -0.2])
                    q[joint] = root
                    self.assertLess(abs(determinant(q)), 2e-16)
                    minus, plus = q.copy(), q.copy()
                    minus[joint] -= 1e-6
                    plus[joint] += 1e-6
                    self.assertLess(determinant(minus) * determinant(plus), 0)
            q = np.array([0.31, -0.5, 1.1, -0.45, 0.9, -0.2])
            a = -upper - forearm * math.cos(q[2]) + effective * math.sin(q[2] + q[3])
            b = forearm * math.sin(q[2]) + effective * math.cos(q[2] + q[3])
            q[1] = math.atan2(-a, b)
            self.assertLess(abs(shoulder(q, geometry)), 2e-16)
            self.assertLess(abs(determinant(q)), 2e-16)
            minus, plus = q.copy(), q.copy()
            minus[1] -= 1e-6
            plus[1] += 1e-6
            self.assertLess(determinant(minus) * determinant(plus), 0)

    def test_controller_assembly_mapping_and_mirrored_defaults(self):
        rng = np.random.default_rng(861)
        max_pose_error = 0.0
        max_jacobian_error = 0.0
        for station in ("diagonal-07", "diagonal-08"):
            directory = ASSETS / "stations" / station
            config = json.loads((directory / "control.json").read_text())
            self.assertEqual(config["joint_mapping"],
                             "q_assembly = q_controller; no baked initial offsets")
            assembly = pin.buildModelFromUrdf(str(directory / config["assembly_urdf"]))
            assembly_data = assembly.createData()
            for arm, entry in config["arms"].items():
                path = directory / entry["control_urdf"]
                control = pin.buildModelFromUrdf(str(path))
                control_data = control.createData()
                geometry = coefficients(path)
                q0 = entry["initial_pose"]
                actual_signs = np.sign([shoulder(q0, geometry), math.sin(q0[2]),
                                       math.sin(q0[4])]).astype(int).tolist()
                self.assertEqual(actual_signs, [-1, 1, 1] if arm == "left" else [1, -1, -1])
                mount = np.array(entry["world_from_base"])
                self.assertAlmostEqual(np.linalg.det(mount[:3, :3]), 1.0, places=14)
                block_rotation = np.zeros((6, 6))
                block_rotation[:3, :3] = block_rotation[3:, 3:] = mount[:3, :3]
                control_frame = control.getFrameId("tool0")
                assembly_frame = assembly.getFrameId(f"{arm}_tool0")
                assembly_joints = [
                    assembly.joints[assembly.getJointId(f"{arm}_{name}")]
                    for name in JOINT_NAMES
                ]
                for _ in range(100):
                    q = rng.uniform(-math.pi, math.pi, 6)
                    assembly_q = pin.neutral(assembly)
                    for value, joint in zip(q, assembly_joints, strict=True):
                        assembly_q[joint.idx_q] = value
                    pin.framesForwardKinematics(control, control_data, q)
                    pin.framesForwardKinematics(assembly, assembly_data, assembly_q)
                    expected_pose = mount @ control_data.oMf[control_frame].homogeneous
                    actual_pose = assembly_data.oMf[assembly_frame].homogeneous
                    max_pose_error = max(max_pose_error, np.max(abs(expected_pose - actual_pose)))
                    np.testing.assert_allclose(actual_pose, expected_pose, atol=1e-14, rtol=0)
                    jc = pin.computeFrameJacobian(control, control_data, q, control_frame,
                                                  pin.LOCAL_WORLD_ALIGNED)
                    ja = pin.computeFrameJacobian(assembly, assembly_data, assembly_q,
                                                  assembly_frame, pin.LOCAL_WORLD_ALIGNED)
                    ja = ja[:, [joint.idx_v for joint in assembly_joints]]
                    expected_jacobian = block_rotation @ jc
                    max_jacobian_error = max(
                        max_jacobian_error, np.max(abs(ja - expected_jacobian))
                    )
                    np.testing.assert_allclose(ja, expected_jacobian, atol=1e-14, rtol=0)
        print(f"400 mounted FK/Jacobian comparisons: max pose {max_pose_error:.3g}, "
              f"max Jacobian {max_jacobian_error:.3g}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
