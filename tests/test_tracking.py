"""Joint-limited tracking: the output follows a moving IK reference without lag."""
import json
import unittest
from pathlib import Path

import numpy as np
from libpyelite import _native
from libpyelite.control_profile import load_recipe
from libpyelite.sim import StationSimulation

ROOT = Path(__file__).resolve().parents[3]
DT = 0.004


def settings(**overrides):
    values = load_recipe(ROOT / "recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml")
    values.update(json.loads((Path(__file__).resolve().parents[1] / "examples/sim_safety.json").read_text()))
    values.update(overrides)
    return values


def drag(sim, ticks=125, distance=(0.0, -0.15, -0.05), ramp=31):
    """Slider-like drag: a new target every 16 ms for ``ramp`` ticks, then hold."""
    home = sim.targets()["left"].copy()
    q, lag, passed = [sim.q[:6].copy()], [], []
    for k in range(ticks):
        target = home.copy()
        target[:3, 3] += np.asarray(distance) * min(1.0, k / ramp)
        sim.set_target("left", target)
        for _ in range(4):
            sim.step()
            q.append(sim.q[:6].copy())
            sender = sim.arms["left"].sender
            passed.append(sender is not None and sender.passed())
        lag.append(np.linalg.norm(sim.tcp_poses()["left"][:3, 3] - target[:3, 3]))
    return np.array(q), np.array(lag), np.array(passed)


class TrackingTests(unittest.TestCase):
    def test_moving_target_has_no_steady_lag_and_limits_hold(self):
        # This ~0.32 m/s reference must be feasible independently of the
        # deployment recipe's more conservative 0.25 m/s cap.
        sim = StationSimulation(settings=settings(max_lin_speed=0.5))
        q, lag, _ = drag(sim)
        # Constant-speed segment (~0.32 m/s): a stop-at-each-target limiter lagged ~6 mm
        # here and the old 0.8/2/10 limits ~180 mm.
        self.assertLess(np.median(lag[12:30]), 0.003)
        self.assertLess(lag[-1], 1e-3)
        config = sim.arms["left"].config
        for n, limit in ((1, config.max_joint_velocity), (2, config.max_joint_acceleration), (3, config.max_joint_jerk)):
            derivative = np.abs(np.diff(q, n=n, axis=0)) / DT**n
            self.assertTrue(np.all(derivative <= np.asarray(limit) * (1 + 1e-6) + 1e-6), (n, derivative.max()))

    def _path(self, motion):
        from libpyelite.sim import _pose_from_xyz_rpy, pose_to_xyz_rpy

        sim = StationSimulation(settings=settings())
        home = sim.targets()["left"].copy()
        start = pose_to_xyz_rpy(home)
        poses = []
        for k in range(91):
            sim.set_target("left", _pose_from_xyz_rpy(start + np.asarray(motion) * min(1.0, k / 31)))
            for _ in range(4):
                sim.step()
                poses.append(sim.tcp_poses()["left"])
        return home, sim.targets()["left"], np.array(poses)

    def test_faster_target_lags_at_the_recipe_cap_then_catches_up(self):
        sim = StationSimulation(settings=settings())
        self.assertEqual(sim.arms["left"].config.max_linear_velocity, 0.25)
        _, lag, _ = drag(sim)
        # The ~0.32 m/s reference exceeds the deployment cap. Lag during the
        # drag is necessary; it must disappear after the target stops.
        self.assertGreater(np.median(lag[12:30]), 0.01)
        self.assertLess(lag[-1], 1e-3)

    def test_translation_stays_on_the_straight_line(self):
        # Per-joint limiting bent this 15 cm move 28 mm off the line.
        home, final, poses = self._path([0.15, 0, 0, 0, 0, 0])
        direction = (final[:3, 3] - home[:3, 3]) / np.linalg.norm(final[:3, 3] - home[:3, 3])
        offset = poses[:, :3, 3] - home[:3, 3]
        lateral = np.linalg.norm(offset - np.outer(offset @ direction, direction), axis=1)
        self.assertLess(lateral.max(), 0.002)
        np.testing.assert_allclose(poses[-1], final, atol=1e-3)

    def test_rotation_turns_about_the_tcp(self):
        # Per-joint limiting swung the TCP point 65 mm during this 0.8 rad yaw.
        home, final, poses = self._path([0, 0, 0, 0, 0, 0.8])
        self.assertLess(np.linalg.norm(poses[:, :3, 3] - home[:3, 3], axis=1).max(), 0.001)
        np.testing.assert_allclose(poses[-1], final, atol=1e-3)

    def test_arrival_does_not_ring(self):
        # A low Cartesian jerk limit against the 20 1/s correction gain rang
        # +16/-11/+8 mm about the target for 0.7 s after the slider stopped.
        home, final, poses = self._path([0.15, 0, 0, 0, 0, 0])
        along = (poses[124:, :3, 3] - final[:3, 3]) @ ((final - home)[:3, 3] / np.linalg.norm((final - home)[:3, 3]))
        significant = np.sign(along[np.abs(along) > 1e-4])
        self.assertLessEqual(int(np.sum(significant[1:] != significant[:-1])), 1)  # at most one overshoot-and-return
        self.assertTrue(np.all(np.abs(along[125:]) < 1e-4))  # within 0.1 mm 0.5 s after the stop

    def test_sender_passes_feasible_planner_samples_through(self):
        sim = StationSimulation(settings=settings())
        _, _, passed = drag(sim, ticks=40)
        # Two limiters in series each add lag; the sender only gates.
        self.assertTrue(passed.all())

    def test_gate_tracks_a_repeated_sample_within_limits(self):
        limits = ([2.0] * 6, [10.0] * 6, [500.0] * 6)
        upstream = _native.MotionLimiter(DT, *limits, [-3.0] * 6, [3.0] * 6)
        sender = _native.MotionLimiter(DT, *limits, [-3.0] * 6, [3.0] * 6)
        upstream.seed([0.0] * 6)
        sender.seed([0.0] * 6)
        sent = [np.zeros(6)]
        for i in range(120):
            q = upstream.follow([1.0] * 6, [1.0] * 6, 20.0)
            if i == 60:
                continue  # a planner tick the sender never saw
            sent.append(np.asarray(sender.gate(q, upstream.velocity(), upstream.acceleration())))
            if i == 61:
                self.assertFalse(sender.passed())
        sent = np.array(sent)
        self.assertLessEqual(np.abs(np.diff(sent, axis=0)).max() / DT, 2.0 * 1.05)
        self.assertLessEqual(np.abs(np.diff(sent, n=2, axis=0)).max() / DT**2, 10.0 * 1.05)

    def test_follow_brakes_before_a_position_limit(self):
        limiter = _native.MotionLimiter(DT, [3.0] * 6, [10.0] * 6, [500.0] * 6, [-0.5] * 6, [0.5] * 6)
        limiter.seed([0.0] * 6)
        for _ in range(300):
            q = np.asarray(limiter.follow([0.5] * 6, [3.0] * 6, 20.0))
            self.assertTrue(np.all(q <= 0.5))
        self.assertGreater(q.min(), 0.49)

    def test_reanchor_keeps_an_active_target(self):
        sim = StationSimulation(settings=settings())
        arm = sim.arms["left"]
        target = sim.targets()["left"].copy()
        target[1, 3] -= 0.05
        sim.set_target("left", target)
        for _ in range(20):
            sim.step()
        status = arm.planner.status()
        arm.planner.reanchor(arm.q, status.joint_velocity, status.joint_acceleration)
        self.assertTrue(arm.planner.status().moving)
        for _ in range(200):
            sim.step()
        self.assertLess(np.linalg.norm(sim.tcp_poses()["left"][:3, 3] - target[:3, 3]), 1e-3)

    def test_set_reference_only_between_plan_and_advance(self):
        sim = StationSimulation(settings=settings())
        planner = sim.arms["left"].planner
        with self.assertRaises(RuntimeError):
            planner.set_reference(planner.q())


if __name__ == "__main__":
    unittest.main()
