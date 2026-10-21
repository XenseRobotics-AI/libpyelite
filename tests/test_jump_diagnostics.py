"""Exercise production jump guards without drivers, sockets or robot motion."""

import re
import unittest
from types import SimpleNamespace

import numpy as np
from libpyelite import EliteDriver, EliteDriverConfig, PinkIKOptions, PlannerConfig, _native, model_path
from libpyelite.control_profile import configure_native_driver
from libpyelite.sim import LEFT


def planner(config=None):
    if config is None:
        config = PlannerConfig()
        config.max_target_translation_jump = 0.1
        config.max_target_rotation_jump = 0.3
    p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), config)
    p.seed(LEFT)
    return p


def target_from(p, translation=0.0, rotation=0.0):
    pose = np.array(p.pose()).reshape(4, 4)
    pose[0, 3] += translation
    turn = np.array(_native.pose_from_rotvec([0, 0, 0, 0, 0, rotation])).reshape(4, 4)
    pose[:3, :3] = turn[:3, :3] @ pose[:3, :3]
    return pose


class JumpDiagnosticTests(unittest.TestCase):
    def reject(self, p, target, sequence=1):
        old_sequence = p.status().sequence
        old_q, old_pose = p.q(), p.pose()
        with self.assertRaisesRegex(ValueError, "Cartesian target jump rejected") as caught:
            p.submit(target.ravel(), sequence * 0.004, sequence)
        self.assertEqual(p.status().sequence, old_sequence)
        np.testing.assert_array_equal(p.q(), old_q)
        np.testing.assert_array_equal(p.pose(), old_pose)
        message = str(caught.exception)
        self.assertIn(f"sequence={sequence} last_accepted_sequence={old_sequence}", message)
        return message

    def check_reference(self, message, name, translation, rotation, exceeded):
        match = re.search(
            rf"; {name}=\{{translation=(\S+) m, rotation=(\S+) rad, exceeded=\[([^\]]*)\]\}}",
            message,
        )
        self.assertIsNotNone(match, message)
        self.assertAlmostEqual(float(match[1]), translation)
        self.assertAlmostEqual(float(match[2]), rotation)
        self.assertEqual(match[3].split(", "), ["native_max_target_" + item + "_jump" for item in exceeded])

    def test_translation_rotation_and_combined_rejections(self):
        for distance, angle, exceeded in (
            (0.2, 0.0, ["translation"]),
            (0.0, 0.5, ["rotation"]),
            (0.2, 0.5, ["translation", "rotation"]),
        ):
            with self.subTest(exceeded=exceeded):
                p = planner()
                message = self.reject(p, target_from(p, distance, angle))
                self.assertIn("arm=unspecified", message)
                self.assertIn("previous_reference=planned_pose", message)
                for key, unit, expected in (("translation", "m", 0.1), ("rotation", "rad", 0.3)):
                    value = re.search(rf"native_max_target_{key}_jump=(\S+) {unit}", message)
                    self.assertIsNotNone(value)
                    self.assertEqual(float(value[1]), expected)
                for name in ("previous", "planned_pose"):
                    self.check_reference(message, name, distance, angle, exceeded)

    def test_two_references_can_trigger_different_limits(self):
        p = planner()
        p.submit(target_from(p, 0.04, 0.2).ravel(), 0.004, 1)
        message = self.reject(p, target_from(p, 0.12, -0.2), sequence=2)
        self.assertIn("previous_reference=previous_target", message)
        self.check_reference(message, "previous", 0.08, 0.4, ["rotation"])
        self.check_reference(message, "planned_pose", 0.12, 0.2, ["translation"])
        # Rejection did not consume sequence 2 or replace the old target.
        p.submit(target_from(p).ravel(), 0.008, 2)
        self.assertEqual(p.status().sequence, 2)

    def test_disabled_dimension_is_not_reported_as_exceeded(self):
        for disabled, enabled in (("translation", "rotation"), ("rotation", "translation")):
            with self.subTest(disabled=disabled):
                c = PlannerConfig()
                setattr(c, "max_target_" + enabled + "_jump", 0.1)
                p = planner(c)
                message = self.reject(p, target_from(p, 0.2, 0.5))
                self.assertRegex(message, rf"native_max_target_{disabled}_jump=0 (m|rad) \(disabled\)")
                self.check_reference(message, "planned_pose", 0.2, 0.5, [enabled])

    def test_disabled_guards_and_timed_reset_still_accept_large_targets(self):
        disabled = planner(PlannerConfig())
        disabled.submit(target_from(disabled, 1.0, 1.0).ravel(), 0.004, 1)
        timed = planner()
        timed.submit(target_from(timed, 1.0, 1.0).ravel(), 0.004, 1, 3.0)
        self.assertEqual(disabled.status().sequence, 1)
        self.assertEqual(timed.status().sequence, 1)

    def test_reanchor_and_small_stream_steps_remain_accepted(self):
        p = planner()
        for step in range(1, 11):
            # Far from planned output, but close to the previous accepted target.
            p.submit(target_from(p, step * 0.04, step * 0.1).ravel(), step * 0.004, step)
        # Far from previous target, but back on planned output: still accepted.
        p.submit(target_from(p).ravel(), 0.044, 11)
        self.assertEqual(p.status().sequence, 11)

    def test_translation_boundary_is_strict_and_reports_small_excess(self):
        c = PlannerConfig()
        p = planner(c)
        here = np.array(p.pose()).reshape(4, 4)
        target = target_from(p, 0.125)
        c.max_target_translation_jump = target[0, 3] - here[0, 3]
        p = planner(c)
        p.submit(target.ravel(), 0.004, 1)  # exactly at the limit
        p.seed(LEFT)  # discard previous target, retaining the accepted sequence
        # Use the delta's ULP: one ULP of the coordinate can disappear when
        # subtracting the reference coordinate to compute the displacement.
        target[0, 3] += 2 * np.spacing(c.max_target_translation_jump)
        message = self.reject(p, target, sequence=2)
        delta = float(re.search(r"previous=\{translation=(\S+) m", message)[1])
        self.assertGreater(delta, c.max_target_translation_jump)

    def test_joint_rejection_reports_all_offending_axes_and_reference(self):
        c = PlannerConfig()
        c.arm_name = "right"
        c.max_target_joint_jump = 0.1
        for has_previous in (False, True):
            with self.subTest(has_previous=has_previous):
                p = planner(c)
                reference = LEFT.copy()
                if has_previous:
                    reference += 0.05
                    p.submit_joints(reference, 0.004, 1)
                requested = reference.copy()
                requested[[1, 4]] += [0.2, -0.25]
                sequence = 2 if has_previous else 1
                with self.assertRaisesRegex(ValueError, "Joint target jump rejected") as caught:
                    p.submit_joints(requested, 0.008, sequence)
                message = str(caught.exception)
                self.assertIn(f"arm=right sequence={sequence} last_accepted_sequence={sequence - 1}", message)
                self.assertIn("reference=" + ("previous_joint_target" if has_previous else "planned_q"), message)
                limit = re.search(r"native_max_target_joint_jump=(\S+) rad", message)
                self.assertEqual(float(limit[1]), 0.1)
                axes = re.findall(r"J(\d)\{delta=(\S+) rad, target=(\S+) rad, reference=(\S+) rad\}", message)
                self.assertEqual([row[0] for row in axes], ["2", "5"])
                for row, expected in zip(axes, (0.2, 0.25), strict=True):
                    index = int(row[0]) - 1
                    self.assertAlmostEqual(float(row[1]), expected)
                    self.assertEqual(float(row[2]), requested[index])
                    self.assertEqual(float(row[3]), reference[index])
                self.assertEqual(p.status().sequence, sequence - 1)
                np.testing.assert_array_equal(p.q(), LEFT)

    def test_arm_identity_survives_profile_and_python_error_bridge(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                cfg = EliteDriverConfig()
                configure_native_driver(cfg, {"native_max_target_translation_jump": 0.1}, side=side)
                p = planner(cfg.planner)
                message = self.reject(p, target_from(p, 0.2))
                self.assertIn(f"arm={side} ", message)
                driver = EliteDriver.__new__(EliteDriver)
                driver._native = SimpleNamespace(status=lambda: SimpleNamespace(error=message))
                with self.assertRaises(RuntimeError) as caught:
                    driver.check_error()
                self.assertEqual(str(caught.exception), "Elite native servo failed: " + message)


if __name__ == "__main__":
    unittest.main()
