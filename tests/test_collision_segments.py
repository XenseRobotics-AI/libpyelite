"""Whole-segment certificates: endpoints and dense samples alone are insufficient."""

import hashlib
import json
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin
from libpyelite import _native
from libpyelite.collision import StationCollision


def joint(name, parent="world", kind="prismatic", origin="0 0 0", axis="1 0 0"):
    return (
        f'<link name="{name}"/><joint name="{name}_joint" type="{kind}">'
        f'<parent link="{parent}"/><child link="{name}"/><origin xyz="{origin}"/>'
        f'<axis xyz="{axis}"/><limit lower="-100" upper="100" effort="1" velocity="100"/></joint>'
    )


def box(frame, xyz=(0, 0, 0), size=(0.1, 0.1, 0.1)):
    s = _native.CollisionShape()
    s.frame, s.type, s.xyz, s.size = frame, 0, list(xyz), list(size)
    return s


class CollisionSegmentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def model(self, joints, shapes, margin=0.01, detection=0.05, buffer=0.001, pairs=None):
        path = Path(self.temp.name) / "segment.urdf"
        path.write_text(f'<robot name="segment"><link name="world"/>{joints}</robot>')
        self.pin = pin.buildModelFromUrdf(str(path))
        pairs = [[0, i] for i in range(1, len(shapes))] if pairs is None else pairs
        return _native.StationCollisionModel(str(path), shapes, pairs, [margin] * len(pairs), detection, 0.2, buffer)

    def linearize(self, model, q):
        return model.linearize(q, list(range(self.pin.nv)))

    def public_model(self, joints, shapes, margin=0.01):
        self.model(joints, shapes, margin=margin)
        urdf = Path(self.temp.name) / "segment.urdf"
        profile = {
            "schema": 1, "margin_m": margin, "detection_distance_m": 0.05,
            "gain": 0.2, "allowed_link_pairs": [],
            "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
            "shapes": [
                {"name": str(i), "frame": s.frame, "group": str(i), "type": "box",
                 "size": s.size, "xyz": s.xyz, "rpy": s.rpy}
                for i, s in enumerate(shapes)
            ],
        }
        path = Path(self.temp.name) / "collision.json"
        path.write_text(json.dumps(profile))
        return StationCollision(self.pin, path, urdf)

    def test_endpoint_safe_prismatic_boxes_must_not_tunnel(self):
        c = self.model(joint("left") + joint("right"), [box("left"), box("right")])
        start = np.array([-0.2, 0.2])
        self.assertTrue(c.check(start).safe)
        self.assertTrue(c.check(-start).safe)
        self.assertEqual(len(self.linearize(c, start)[3]), 0)  # starts outside the band
        self.assertFalse(c.verify_step(start, -start).safe)

    def test_thin_obstacle_between_coarse_samples(self):
        # Collision at a non-dyadic t; even 2 mm spacing can miss this entirely.
        tiny = (1e-5, 0.01, 0.01)
        for position in (0.371123456, 0.937123456):
            with self.subTest(position=position):
                c = self.model(joint("moving"), [box("moving", size=tiny), box("world", (position, 0, 0), tiny)],
                               margin=1e-5, buffer=0)
                start, end = [0.0], [1.0]
                self.assertTrue(c.check(start).safe)
                self.assertTrue(c.check(end).safe)
                self.linearize(c, start)
                self.assertFalse(c.verify_step(start, end, resolution=2).safe)

    def test_between_sample_margin_violation_without_shape_contact(self):
        tiny = (1e-5, 0.01, 0.01)
        c = self.model(joint("moving"), [box("moving", size=tiny),
                                       box("world", (0.371123456, 0.010005, 0), tiny)],
                       margin=1e-5, buffer=0)
        self.assertTrue(c.check([0]).safe)
        self.assertTrue(c.check([1]).safe)
        middle = c.check([0.371123456])
        self.assertGreater(middle.minimum_distance, 0)
        self.assertFalse(middle.safe)
        self.linearize(c, [0])
        self.assertFalse(c.verify_step([0], [1], resolution=2).safe)

    def test_public_path_checker_cannot_skip_thin_out_of_band_obstacle(self):
        tiny = (1e-5, 0.01, 0.01)
        c = self.public_model(joint("moving"), [box("moving", size=tiny),
                                              box("world", (0.371123456, 0, 0), tiny)], margin=1e-5)
        self.assertFalse(c.check_segment([0], [1], resolution=2, include_start=False, cutoff=0.001).safe)

    def test_public_path_checker_restores_linearization_and_checks_start(self):
        c = self.public_model(joint("moving"), [box("moving"), box("world", (0.14, 0, 0))])
        c.linearize([-0.2], [0])
        self.assertTrue(c.check_segment([-0.4], [-0.3]).safe)
        self.assertTrue(c.verify_step([-0.2], [-0.21]).safe)
        self.assertFalse(c.check_segment([0.14], [-0.4], include_start=False).safe)
        self.assertTrue(c.verify_step([-0.2], [-0.21]).safe)
        for kwargs in ({"resolution": np.nan}, {"cutoff": np.inf}):
            with self.assertRaises(ValueError):
                c.check_segment([0], [1], **kwargs)

    def test_safe_large_translation_is_certified(self):
        c = self.model(joint("moving"), [box("moving"), box("world", (0.2, 0.3, 0))])
        self.linearize(c, [-1.0])
        self.assertTrue(c.verify_step([-1.0], [1.0]).safe)
        self.assertGreater(c.last_sampled_pairs, 0)
        self.assertLessEqual(c.last_narrow_phase_calls, 4096)

    def test_large_serial_rotations_cross_obstacle(self):
        joints = joint("first", kind="revolute", axis="0 0 1") + joint(
            "second", parent="first", kind="revolute", origin="1 0 0", axis="0 0 1"
        )
        c = self.model(joints, [box("second", (1, 0, 0), (0.02,) * 3), box("world", (2, 0, 0), (0.02,) * 3)])
        start = np.array([-np.pi / 2, np.pi / 2])
        self.assertTrue(c.check(start).safe)
        self.assertTrue(c.check(-start).safe)
        self.linearize(c, start)
        self.assertFalse(c.verify_step(start, -start).safe)

    def test_folded_chain_and_downstream_prismatic_extension(self):
        joints = joint("rotate", kind="revolute", axis="0 0 1") + joint("extend", parent="rotate")
        # Radius grows from .1 to 2 during the turn: q0 lever arm is insufficient.
        t = 0.625
        target = (0.1 + 1.9 * t) * np.array([np.cos(np.pi * t), np.sin(np.pi * t), 0])
        c = self.model(joints, [box("extend", size=(0.02,) * 3), box("world", target, (0.02,) * 3)])
        start, end = [0, 0.1], [np.pi, 2]
        self.assertTrue(c.check(start).safe)
        self.assertTrue(c.check(end).safe)
        self.linearize(c, start)
        self.assertFalse(c.verify_step(start, end).safe)

    def test_large_safe_serial_rotations_are_certified(self):
        joints = joint("first", kind="revolute", axis="0 0 1") + joint(
            "second", parent="first", kind="revolute", origin="1 0 0", axis="0 0 1"
        )
        c = self.model(joints, [box("second", (1, 0, 0)), box("world", (0, 4, 0))])
        start, end = [-2.0, -2.0], [2.0, 2.0]
        self.linearize(c, start)
        self.assertTrue(c.verify_step(start, end).safe)

    def test_unaligned_joint_axes(self):
        for kind, axis, local, obstacle, start, end in [
            ("prismatic", "0.6 0.8 0", (0, 0, 0), (0.3, 0.4, 0), [0], [1]),
            ("revolute", "0.6 0.8 0", (0, 0, 1), (0, 0, 1), [-1], [1]),
        ]:
            with self.subTest(kind=kind):
                c = self.model(joint("moving", kind=kind, axis=axis),
                               [box("moving", local), box("world", obstacle)])
                self.assertTrue(c.check(start).safe)
                self.assertTrue(c.check(end).safe)
                self.linearize(c, start)
                self.assertFalse(c.verify_step(start, end).safe)

    def test_no_usable_detection_band_uses_full_certificate(self):
        c = self.model(joint("moving"), [box("moving"), box("world", (0.14, 0, 0))],
                       margin=0.03, detection=0.02, buffer=0)
        self.assertEqual(len(self.linearize(c, [0])[3]), 0)
        self.assertTrue(c.verify_step([0], [-0.1]).safe)
        self.assertFalse(c.verify_step([0], [0.3]).safe)

    def test_configuration_indices_differ_from_velocity_indices(self):
        joints = joint("a_unused", kind="continuous", axis="0 0 1") + joint("z_slide")
        c = self.model(joints, [box("z_slide"), box("world")])
        self.assertEqual((self.pin.nq, self.pin.nv), (3, 2))
        start, end = [1, 0, -0.2], [1, 0, 0.2]
        self.linearize(c, start)
        self.assertFalse(c.verify_step(start, end).safe)

    def test_unsupported_moving_joint_fails_explicitly(self):
        c = self.model(joint("moving", kind="continuous", axis="0 0 1"),
                       [box("moving", (1, 0, 0)), box("world", (0, 2, 0))])
        self.linearize(c, [1, 0])
        with self.assertRaisesRegex(ValueError, "scalar revolute/prismatic"):
            c.verify_step([1, 0], [0, 1])

    def test_convex_radius_about_offset_origin_covers_rotation(self):
        # Off-centre convex vertices exercise the geometry worker's radius contract.
        shape = box("moving")
        shape.type = 3
        shape.vertices = [[x, y, z] for x in (0.99, 1.01) for y in (-0.01, 0.01) for z in (-0.01, 0.01)]
        # Outward triangulated cube, with vertices ordered x/y/z.
        shape.faces = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
                       [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                       [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
        c = self.model(joint("moving", kind="revolute", axis="0 0 1"),
                       [shape, box("world", (1, 0, 0), (0.01,) * 3)])
        self.linearize(c, [-1.0])
        self.assertFalse(c.verify_step([-1.0], [1.0]).safe)

    def test_budget_exhaustion_is_unsafe_even_for_clear_samples(self):
        # Safe translation tangent to the margin cannot beat the 1 um reserve.
        # The depth/work budget must reject it, never skip the pending interval.
        c = self.model(joint("moving"), [box("moving"), box("world", (0, 0.11, 0), (100, 0.1, 0.1))])
        start, end = [-0.2], [0.2]
        self.assertTrue(c.check(start).safe)
        self.assertTrue(c.check(end).safe)
        self.linearize(c, start)
        before = time.monotonic()
        result = c.verify_step(start, end)
        self.assertFalse(result.safe)
        self.assertLess(result.clearance, 0)
        self.assertLess(time.monotonic() - before, 2)
        self.assertLessEqual(c.last_narrow_phase_calls, 4096)
        self.assertTrue(c.check(start).safe)  # unsafe is conservative, not measured penetration

    def test_nonzero_near_margin_tangential_motion_is_certified(self):
        c = self.model(joint("moving"), [box("moving"), box("world", (0, 0.1101, 0), (1, 0.1, 0.1))])
        self.linearize(c, [-0.002])
        result = c.verify_step([-0.002], [0.002])
        self.assertTrue(result.safe)
        self.assertGreater(c.last_sampled_pairs, 0)
        self.assertGreater(c.last_narrow_phase_calls, 0)
        self.assertGreaterEqual(result.clearance, 0)

    def test_large_safe_motion_budget_is_not_silently_approved(self):
        c = self.model(joint("moving"), [box("moving"), box("world", (0, 0.12, 0), (100, 0.1, 0.1))])
        self.linearize(c, [-10.0])
        self.assertFalse(c.verify_step([-10.0], [10.0]).safe)
        self.assertLessEqual(c.last_narrow_phase_calls, 4096)

    def test_small_step_uses_cached_certificate_and_repeated_start(self):
        c = self.model(joint("moving"), [box("moving"), box("world", (0.14, 0, 0))])
        self.linearize(c, [0])
        self.assertTrue(c.verify_step([0], [0.001]).safe)
        self.assertEqual(c.last_narrow_phase_calls, 0)
        self.assertEqual(c.last_sampled_pairs, 0)
        self.assertFalse(c.verify_step([0], [0.3]).safe)
        self.assertTrue(c.verify_step([0], [-0.001]).safe)
        self.assertEqual(c.last_narrow_phase_calls, 0)

    def test_invalid_requests_and_missing_linearization(self):
        c = self.model(joint("moving"), [box("moving"), box("world", (0.3, 0, 0))])
        with self.assertRaises(RuntimeError):
            c.verify_step([0], [0.01])
        self.linearize(c, [0])
        for start, end, resolution in [([np.nan], [0], .002), ([0], [np.inf], .002),
                                       ([], [0], .002), ([0], [], .002), ([0], [0], np.nan),
                                       ([0], [0], np.inf), ([0], [0], 0), ([0], [0], -1)]:
            with self.subTest(start=start, end=end, resolution=resolution), self.assertRaises(ValueError):
                c.verify_step(start, end, resolution)
        with self.assertRaises(RuntimeError):
            c.verify_step([1e-13], [0.01])
        self.assertTrue(c.verify_step([0], [0]).safe)

    def test_finite_endpoints_with_overflowing_displacement_are_rejected(self):
        c = self.model(joint("moving"), [box("moving"), box("world")])
        self.linearize(c, [-1e308])
        with self.assertRaisesRegex(ValueError, "Nonfinite collision displacement"):
            c.verify_step([-1e308], [1e308])

    def test_zero_dof_empty_pairs(self):
        c = self.model("", [box("world")], pairs=[])
        self.linearize(c, [])
        self.assertTrue(c.verify_step([], []).safe)


if __name__ == "__main__":
    unittest.main()
