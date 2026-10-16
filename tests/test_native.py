"""Hardware-free regression suite; never constructs NativeDriver or opens sockets."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from libpyelite import PinkIKModel, PinkIKOptions, PlannerConfig, model_path, _native


LEFT = np.array([-1.3963, -0.5236, 1.5708, -0.5236, 1.5708, -0.7854])
RIGHT = np.array([-1.7453, -2.6180, -1.5708, -2.6180, -1.5708, -0.7854])


class ModelTests(unittest.TestCase):
    def setUp(self):
        self.model = PinkIKModel()

    def test_official_yaml_matches_urdf(self):
        # Only the flat per-joint numeric fields are needed; no PyYAML dependency.
        source = Path(model_path()).parents[1] / "joint_limits.yaml"
        joints, current = {}, None
        for line in source.read_text().splitlines():
            if line.startswith("  ") and not line.startswith("    ") and line.endswith(":"):
                current = line.strip().removesuffix(":")
                joints[current] = {}
            elif current and ": !degrees" in line:
                key, value = line.strip().split(": !degrees")
                joints[current][key] = math.radians(float(value))
        root = ET.parse(model_path()).getroot()
        active = [j for j in root.findall("joint") if j.attrib["type"] == "revolute"]
        self.assertEqual(len(active), 6)
        self.assertEqual(len(joints), 6)
        for joint in active:
            expected = joints[joint.attrib["name"]]
            limits = joint.find("limit").attrib
            for xml_key, yaml_key in (("lower", "min_position"), ("upper", "max_position"),
                                      ("velocity", "max_velocity")):
                self.assertAlmostEqual(float(limits[xml_key]), expected[yaml_key], places=12)
        np.testing.assert_allclose(self.model.upper_limits, [2*math.pi]*2+[math.pi]+[2*math.pi]*3)
        np.testing.assert_allclose(self.model.velocity_limits, [math.pi]*6)

    def test_wheel_meshes_exist(self):
        urdf = Path(model_path())
        for mesh in ET.parse(urdf).getroot().iter("mesh"):
            self.assertTrue((urdf.parent / mesh.attrib["filename"]).is_file())

    def test_defaults(self):
        self.assertEqual(PinkIKOptions().max_iterations, 500)
        # One failed write faults unless the adapter widens the tolerance.
        self.assertEqual(_native.EliteDriverConfig().max_consecutive_write_failures, 1)

    def test_fk_ik_roundtrip_and_small_motion_both_starts(self):
        for q in (LEFT, RIGHT):
            target = self.model.fk(q)
            solved, d = self.model.solve(target, q)
            self.assertTrue(d.success)
            self.assertEqual(d.iterations, 0)
            np.testing.assert_allclose(solved, q, atol=1e-12)
            target[0, 3] += 0.002
            solved, d = self.model.solve(target, q)
            self.assertTrue(d.success, (d.position_error, d.orientation_error))
            self.assertLessEqual(d.iterations, 500)
            self.assertLess(np.linalg.norm(self.model.fk(solved)[:3, 3]-target[:3, 3]), 1e-4)
            self.assertTrue(np.all(solved >= self.model.lower_limits))
            self.assertTrue(np.all(solved <= self.model.upper_limits))

    def test_tool_transform(self):
        tool = np.eye(4)
        tool[:3, 3] = [0.01, -0.02, 0.15]
        with_tool = PinkIKModel(tool=tool)
        np.testing.assert_allclose(with_tool.fk(LEFT), self.model.fk(LEFT) @ tool, atol=1e-12)

    def test_invalid_fk_is_not_silently_clipped(self):
        for q in ([0, 0, 4, 0, 0, 0], [float("nan")]*6):
            with self.assertRaises(ValueError):
                self.model.fk(q)

    def test_unreachable_target_and_invalid_pose(self):
        target = self.model.fk(LEFT)
        target[0, 3] = 10
        q, d = self.model.solve(target, LEFT)
        self.assertFalse(d.success)
        self.assertTrue(np.isfinite(q).all())
        self.assertTrue(np.all(q >= self.model.lower_limits))
        self.assertTrue(np.all(q <= self.model.upper_limits))
        target[3, 3] = 2
        _, d = self.model.solve(target, LEFT)
        self.assertFalse(d.success)

    def test_rotvec_boundary_roundtrip(self):
        for q in (LEFT, RIGHT):
            matrix = self.model.fk(q)
            pose6 = _native.pose_to_rotvec(matrix.ravel())
            np.testing.assert_allclose(np.array(_native.pose_from_rotvec(pose6)).reshape(4, 4),
                                       matrix, atol=1e-12)


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.model = PinkIKModel()
        self.config = PlannerConfig()
        self.planner = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), self.config)
        self.planner.seed(LEFT)

    def test_latest_target_rebases_and_limits_joint_step(self):
        p = self.planner
        a, b = self.model.fk(LEFT), self.model.fk(LEFT)
        a[0, 3] += 0.02
        b[1, 3] -= 0.01
        p.submit(a.ravel(), 1., 1)
        previous = np.array(p.tick(1.))
        p.submit(b.ravel(), 1.033, 2)
        for i in range(60):
            q = np.array(p.tick(1.033+i*0.004))
            self.assertTrue(np.all(np.abs(q-previous) <= math.pi*0.004+1e-10))
            previous = q
        self.assertEqual(p.status().sequence, 2)
        self.assertEqual(p.status().consumed_sequence, 2)
        self.assertEqual(p.status().ik_failures, 0)
        np.testing.assert_allclose(np.array(p.pose()).reshape(4, 4)[:3, 3], b[:3, 3], atol=1e-4)

    def test_iteration_exhaustion_keeps_bounded_progress(self):
        options = PinkIKOptions()
        options.max_iterations = 1
        planner = _native.Planner(model_path(), np.eye(4).ravel(), options, self.config)
        planner.seed(LEFT)
        target = self.model.fk(LEFT)
        target[0, 3] += .02
        planner.submit(target.ravel(), 0., 1)
        q = np.asarray(planner.tick(0.))
        self.assertFalse(planner.status().ik.success)
        self.assertTrue(planner.status().ik.progress)
        self.assertTrue(planner.status().moving)
        self.assertEqual(planner.status().consumed_sequence, 0)
        self.assertGreater(np.linalg.norm(q - LEFT), 1e-6)
        self.assertTrue(np.all(np.abs(q - LEFT) <= self.model.velocity_limits * .004 + 1e-12))
        for i in range(1, 120):
            q = np.asarray(planner.tick(i * .004))
        self.assertLess(np.linalg.norm(self.model.fk(q)[:3, 3] - target[:3, 3]), 1e-4)
        self.assertEqual(planner.status().consumed_sequence, 1)

    def test_timed_reset_does_not_accept_partial_ik(self):
        options = PinkIKOptions()
        options.max_iterations = 1
        planner = _native.Planner(model_path(), np.eye(4).ravel(), options, self.config)
        planner.seed(LEFT)
        target = self.model.fk(LEFT)
        target[0, 3] += .02
        planner.submit(target.ravel(), 0., 1, .004)
        for i in range(50):
            planner.tick(i * .004)
            if not planner.status().moving:
                break
        self.assertFalse(planner.status().moving)
        self.assertGreater(planner.status().ik_failures, 0)
        self.assertEqual(planner.status().consumed_sequence, 0)

    def test_rotating_stream_and_tool_offset(self):
        from libpyelite.sim import Simulation
        tool = np.eye(4)
        tool[:3, 3] = [.01, -.02, .15]
        sim = Simulation(q=LEFT, tool=tool)
        home = sim.target.copy()
        for i in range(600):
            angle = .12 * math.sin(i * math.pi / 300) if i < 300 else 0.
            rotation = np.asarray(_native.pose_from_rotvec([0, 0, 0, 0, angle, 0])).reshape(4, 4)
            target = home.copy()
            target[:3, :3] = rotation[:3, :3] @ home[:3, :3]
            sample = sim.step(target)
            self.assertFalse(sample["qp_failed"])
            self.assertLessEqual(sample["max_joint_step_rad"], math.pi * .004 + 1e-12)
        self.assertLess(sample["position_error_m"], 1e-4)
        self.assertLess(sample["orientation_error_rad"], 1e-3)

    def test_unreachable_target_recovers_and_stale_still_holds(self):
        from libpyelite.sim import Simulation
        sim = Simulation(q=LEFT)
        home = sim.target.copy()
        far = home.copy()
        far[0, 3] += 2.
        previous = sim.q.copy()
        for i in range(1250):
            sample = sim.step(far if i < 250 else home)
            self.assertFalse(sample["qp_failed"])
            self.assertTrue(np.all(np.abs(sim.q-previous) <= sim.model.velocity_limits*.004+1e-12))
            previous = sim.q.copy()
        self.assertLess(sample["position_error_m"], 1e-4)
        self.assertLess(sample["orientation_error_rad"], 1e-3)
        for _ in range(130):
            sample = sim.step(heartbeat=False)
        self.assertTrue(sample["stale"])
        held = sim.q.copy()
        for _ in range(20):
            sim.step(heartbeat=False)
        np.testing.assert_array_equal(sim.q, held)

    def test_near_singular_stream_and_target_reversal(self):
        from libpyelite.sim import Simulation
        sim = Simulation(q=[-1.4, -.5, 1.5, -.5, .001, -.8])
        home = sim.target.copy()
        for i in range(600):
            target = home.copy()
            target[1, 3] += .03 if i < 200 else -.03 if i < 400 else 0.
            sample = sim.step(target)
            self.assertFalse(sample["qp_failed"])
            self.assertLessEqual(sample["max_joint_step_rad"], math.pi * .004 + 1e-12)
            self.assertTrue(np.all(sim.q >= sim.model.lower_limits))
            self.assertTrue(np.all(sim.q <= sim.model.upper_limits))
        self.assertLess(sample["position_error_m"], .001)

    def test_stale_and_cancel(self):
        p = self.planner
        target = self.model.fk(LEFT)
        p.submit(target.ravel(), 1., 1)
        p.tick(1.)
        p.tick(1.6)
        self.assertTrue(p.status().idle)
        self.assertTrue(p.status().stale)
        p.submit(target.ravel(), 2., 2)
        p.cancel()
        np.testing.assert_allclose(p.tick(2.), LEFT)
        self.assertFalse(p.status().moving)
        self.assertTrue(p.status().idle)

    def test_reset_duration_is_not_input_staleness(self):
        p = self.planner
        target = self.model.fk(LEFT)
        target[0, 3] += 0.001
        p.submit(target.ravel(), 1., 1, 1.)
        for i in range(260):
            p.tick(1.+i*0.004)
        self.assertEqual(p.status().consumed_sequence, 1)
        self.assertFalse(p.status().stale)
        self.assertEqual(p.status().ik_failures, 0)

    def test_joint_stream_gate(self):
        p = self.planner
        goal = LEFT.copy()
        goal[0] += 0.5
        p.submit_joints(goal, 1., 1)
        np.testing.assert_allclose(np.array(p.tick(1.))[0]-LEFT[0], math.pi*0.004)
        with self.assertRaises(ValueError):
            p.submit_joints([0, 0, 4, 0, 0, 0], 1.1, 2)

    def test_invalid_input(self):
        with self.assertRaises(ValueError):
            self.planner.submit([0.]*16, 1., 1)
        with self.assertRaises(ValueError):
            self.planner.submit(np.eye(4).ravel(), 1., 1, -1)
        with self.assertRaises(ValueError):
            cfg = PlannerConfig()
            cfg.velocity_scale = 2
            _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), cfg)

    def test_interpolation_endpoints_and_valid_rotation(self):
        a, b = self.model.fk(LEFT), self.model.fk(RIGHT)
        np.testing.assert_allclose(_native.interpolate_pose(a.ravel(), b.ravel(), 0), a.ravel(), atol=1e-12)
        np.testing.assert_allclose(_native.interpolate_pose(a.ravel(), b.ravel(), 1), b.ravel(), atol=1e-12)
        mid = np.array(_native.interpolate_pose(a.ravel(), b.ravel(), 0.5)).reshape(4, 4)
        np.testing.assert_allclose(mid[:3, :3].T @ mid[:3, :3], np.eye(3), atol=1e-12)
        np.testing.assert_allclose(mid[:3, 3], (a[:3, 3]+b[:3, 3])/2)


if __name__ == "__main__":
    unittest.main()
