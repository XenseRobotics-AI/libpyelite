import unittest

import numpy as np
from libpyelite import PinkIKModel, PinkIKOptions, PlannerConfig, _native, model_path
from libpyelite.sim import LEFT, Simulation


def config():
    c = PlannerConfig()
    c.joint_motion_limits = True
    c.max_joint_velocity = [0.8] * 6
    c.max_joint_acceleration = [2.0] * 6
    c.max_joint_jerk = [10.0] * 6
    return c


class MotionSafetyTests(unittest.TestCase):
    def test_reversal_braking_and_reseed(self):
        p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), config())
        p.seed(LEFT)
        previous_v = np.zeros(6)
        previous_a = np.zeros(6)
        previous_q = LEFT.copy()
        for i in range(1400):
            if i in (0, 250):
                p.submit_joints(LEFT + (0.5 if i == 0 else -0.3), i * 0.004, i + 1)
            if i == 350:
                p.cancel()
            q = np.array(p.tick(i * 0.004))
            s = p.status()
            v = np.array(s.joint_velocity)
            a = np.array(s.joint_acceleration)
            self.assertLessEqual(np.max(abs(v)), 0.8 + 1e-9)
            self.assertLessEqual(np.max(abs(a)), 2 + 1e-9)
            self.assertLessEqual(np.max(abs(a - previous_a)) / 0.004, 10 + 1e-6)
            self.assertLessEqual(np.max(abs(q - previous_q)) / 0.004, 0.8 + 1e-6)
            previous_v, previous_a, previous_q = v, a, q
        self.assertTrue(p.status().idle)
        self.assertLess(np.linalg.norm(previous_v), 1e-10)
        p.seed(LEFT)
        np.testing.assert_array_equal(p.q(), LEFT)

    def test_sender_smoothing_and_timeout_stop(self):
        s = Simulation(planner_config=config())
        target = s.target.copy()
        target[0, 3] += 0.03
        positions = [s.q.copy()] * 3
        for i in range(1000):
            s.step(target, heartbeat=i < 100)
            positions.append(s.q.copy())
        q = np.array(positions)
        dt = s.config.period
        self.assertLessEqual(np.max(abs(np.diff(q, axis=0) / dt)), 0.8 + 1e-8)
        self.assertLessEqual(np.max(abs(np.diff(q, n=2, axis=0) / dt**2)), 2 + 1e-7)
        self.assertLessEqual(np.max(abs(np.diff(q, n=3, axis=0) / dt**3)), 10 + 1e-5)
        self.assertTrue(s.planner.status().stale)
        np.testing.assert_allclose(q[-1], q[-5], atol=1e-12)

    def test_invalid_limits_and_jump_rejection_do_not_consume_target(self):
        c = config()
        c.max_joint_jerk = [0] * 6
        with self.assertRaises(ValueError):
            _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), c)
        c = config()
        c.max_target_translation_jump = 0.05
        c.max_target_joint_jump = 0.1
        p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), c)
        p.seed(LEFT)
        pose = PinkIKModel().fk(LEFT)
        pose[0, 3] += 1
        with self.assertRaises(ValueError):
            p.submit(pose.ravel(), 0.0, 1)
        with self.assertRaises(ValueError):
            p.submit_joints(LEFT + 0.2, 0.0, 1)
        self.assertEqual(p.status().sequence, 0)
        np.testing.assert_array_equal(p.q(), LEFT)

    def test_jump_back_onto_the_arm_is_accepted(self):
        # Teleop re-anchoring after a collision hold: the previous target is far
        # behind the obstacle, the new one is where the arm is. Not a jump.
        c = config()
        c.max_target_translation_jump = 0.05
        p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), c)
        p.seed(LEFT)
        here = PinkIKModel().fk(LEFT)
        away = here.copy()
        for step in range(1, 11):  # a target stream walking 0.2 m away, 2 cm at a time
            away[0, 3] = here[0, 3] + 0.02 * step
            p.submit(away.ravel(), 0.004 * step, step)
        p.submit(here.ravel(), 0.05, 11)  # back onto the arm (which has not moved)
        far = here.copy()
        far[0, 3] += 0.5
        with self.assertRaises(ValueError):
            p.submit(far.ravel(), 0.06, 12)

    def test_nonfinite_and_braking_overshoot_rejected(self):
        p = _native.MotionLimiter(0.004, [1] * 6, [2] * 6, [10] * 6, [-1] * 6, [1] * 6)
        p.seed([0] * 6)
        with self.assertRaises((ValueError, RuntimeError)):
            p.step([np.nan] * 6)
        p.seed([0] * 6)
        with self.assertRaises(RuntimeError):
            p.step([2] * 6)

    def test_native_recipe_rejects_missing_safety_fields_before_connection(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as d:
            recipe = Path(d) / "output.txt"
            recipe.write_text(
                "timestamp\nactual_TCP_pose\nactual_joint_positions\nactual_joint_speeds\nactual_joint_torques\n"
            )
            with self.assertRaisesRegex(ValueError, "robot_mode"):
                _native.RtsiIOInterface(str(recipe), "", 250.0)

    def test_robot_fault_and_wrench_trip_injection(self):
        check = _native.validate_robot_feedback
        check(7, 1, [0] * 6, [30, 30, 30, 5, 5, 5])
        check(7, 2, [0] * 6, [30, 30, 30, 5, 5, 5])
        for mode in (3, 5, 6, 7, 8, 9, -2):
            with self.assertRaises(RuntimeError):
                check(7, mode, [0] * 6, [0] * 6)
        with self.assertRaises(RuntimeError):
            check(5, 1, [0] * 6, [0] * 6)
        with self.assertRaises(RuntimeError):
            check(7, 1, [31, 0, 0, 0, 0, 0], [30, 30, 30, 5, 5, 5])
        with self.assertRaises(RuntimeError):
            check(7, 1, [float("nan")] * 6, [30] * 6)


if __name__ == "__main__":
    unittest.main()
