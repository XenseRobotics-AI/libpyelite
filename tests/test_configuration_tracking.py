"""Offline CS66 boundary regressions; no driver, socket, or robot connection."""
import math
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from libpyelite import PinkIKModel, PinkIKOptions, PlannerConfig, _native, model_path
from libpyelite.assembly import Assembly
from libpyelite.sim import Simulation


def protected_config(**overrides):
    config = PlannerConfig()
    config.preserve_configuration = True
    config.joint_motion_limits = True
    config.cartesian_motion_limits = True
    config.max_joint_velocity = [2.6, 2.6, 3.1, 3.1, 3.1, 3.1]
    config.max_joint_acceleration = [40.0] * 6
    config.max_joint_jerk = [5000.0] * 6
    config.max_linear_velocity = 0.25
    config.max_angular_velocity = 1.0
    config.max_linear_acceleration = 5.0
    config.max_angular_acceleration = 10.0
    config.max_linear_jerk = 1000.0
    config.max_angular_jerk = 2500.0
    for key, value in overrides.items():
        setattr(config, key, value)
    return config


def near_straight(side, angle=0.1):
    q = np.array([0.0, -0.5 - math.atan2(0.3905 * math.sin(angle),
                                      0.427 + 0.3905 * math.cos(angle)),
                  angle, 0.0, math.pi / 2, 0.0])
    q[3] = -1.5 - q[1] - q[2]
    if side == "right":
        q[[0, 1, 3]] = -math.pi - q[[0, 1, 3]]
        q[[2, 4]] *= -1
    return q


def run_target(sim, target, ticks):
    samples = []
    for _ in range(ticks):
        sim.step(target)
        samples.append(sim.q.copy())
        if not sim.planner.configuration_valid(sim.q):
            raise AssertionError(f"Configuration changed: {sim.q}")
    return np.array(samples)


class ConfigurationTrackingTests(unittest.TestCase):
    def test_mirrored_assemblies_capture_each_initial_branch(self):
        for name in ("diagonal-07", "diagonal-08"):
            assembly = Assembly(name)
            for side in ("left", "right"):
                with self.subTest(station=name, side=side):
                    q = assembly.initial_pose(side)
                    p = _native.Planner(assembly.control_model_path(side), np.eye(4).ravel(),
                                        PinkIKOptions(), protected_config())
                    p.seed(q)
                    self.assertTrue(p.configuration_valid(q))
                    lo, hi = np.array(p.command_lower_limits()), np.array(p.command_upper_limits())
                    if side == "left":
                        self.assertGreater(lo[2], 0)
                        self.assertGreater(lo[4], 0)
                    else:
                        self.assertLess(hi[2], 0)
                        self.assertLess(hi[4], 0)
                    # Idle feedback reseeding must not move the winding interval.
                    moved = q.copy()
                    moved[0] += 0.1
                    p.cancel()
                    p.seed(moved)
                    np.testing.assert_array_equal(p.command_lower_limits(), lo)
                    np.testing.assert_array_equal(p.command_upper_limits(), hi)

    def test_invalid_initialization_and_unknown_geometry_fail_without_poisoning_policy(self):
        p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), protected_config())
        with self.assertRaisesRegex(ValueError, "singular|margin"):
            p.seed(np.zeros(6))
        p.seed(near_straight("left"))
        self.assertTrue(p.configuration_valid(p.q()))
        with tempfile.TemporaryDirectory() as directory:
            root = ET.parse(model_path())
            root.getroot().find("./joint[@name='elbow_joint']/axis").set("xyz", "1 0 0")
            path = Path(directory) / "different.urdf"
            root.write(path)
            changed = _native.Planner(str(path), np.eye(4).ravel(), PinkIKOptions(), protected_config())
            with self.assertRaisesRegex(ValueError, "recognized CS66"):
                changed.seed(near_straight("left"))
            root = ET.parse(model_path())
            root.getroot().find("./joint[@name='flange-tool0']/parent").set("link", "forearm_link")
            root.write(path)
            changed = _native.Planner(str(path), np.eye(4).ravel(), PinkIKOptions(), protected_config())
            with self.assertRaisesRegex(ValueError, "recognized CS66"):
                changed.seed(near_straight("left"))

    def test_extend_and_return_preserves_both_elbow_branches(self):
        assembly = Assembly("diagonal-08")
        for side in ("left", "right"):
            with self.subTest(side=side):
                q0 = near_straight(side)
                sim = Simulation(urdf=assembly.control_model_path(side), q=q0,
                                 planner_config=protected_config())
                home = sim.target.copy()
                stretched = sim.model.fk(near_straight(side, 0.0))
                direction = stretched[:3, 3] - home[:3, 3]
                far = home.copy()
                far[:3, 3] += 0.2 * direction / np.linalg.norm(direction)
                outward = run_target(sim, far, 1500)
                sign = 1 if side == "left" else -1
                self.assertGreaterEqual((sign * outward[:, 2]).min(),
                                        sim.config.configuration_margin - 1e-10)
                self.assertTrue(sim.planner.status().boundary_limited)
                self.assertLess(abs(sim.q[2]), abs(q0[2]) - 0.02)
                self.assertLessEqual(
                    np.linalg.norm(_native.pose_to_rotvec(
                        (np.linalg.inv(home) @ sim.model.fk(sim.planner.reference())).ravel())[3:]),
                    sim.config.boundary_orientation_tolerance + 1e-8,
                )
                run_target(sim, home, 2000)
                np.testing.assert_allclose(sim.q, q0, atol=1e-3)
                np.testing.assert_allclose(sim.model.fk(sim.q), home, atol=1e-4)

    def test_prolonged_boundary_hold_does_not_accumulate_return_delay(self):
        q0 = near_straight("left")
        recovered = []
        for hold in (500, 2500):
            sim = Simulation(q=q0, planner_config=protected_config())
            home = sim.target.copy()
            far = home.copy()
            direction = sim.model.fk(near_straight("left", 0))[:3, 3] - home[:3, 3]
            far[:3, 3] += 0.2 * direction / np.linalg.norm(direction)
            run_target(sim, far, hold)
            # Re-grip emits measured TCP as a fresh anchor, then a reachable pull.
            run_target(sim, sim.model.fk(sim.q), 1)
            for tick in range(1250):
                run_target(sim, home, 1)
                if np.max(np.abs(sim.q - q0)) < 1e-3:
                    recovered.append(tick)
                    break
            else:
                self.fail("No recovery within five seconds")
        self.assertLess(abs(recovered[1] - recovered[0]), 100)

    def test_low_jerk_sender_brakes_before_branch_and_allows_retreat(self):
        q0 = near_straight("left", 0.3)
        config = protected_config(max_joint_velocity=[0.6] * 6,
                                  max_joint_acceleration=[1.0] * 6,
                                  max_joint_jerk=[4.0] * 6)
        p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), config)
        p.seed(q0)
        sender = p.make_sender_limiter(q0)
        target = q0.copy()
        target[2] = config.configuration_margin
        velocity = np.zeros(6)
        velocity[2] = -0.6
        previous_a = np.zeros(6)
        q = q0.copy()
        for _ in range(1000):
            q = np.array(sender.follow(target, velocity, 20.0))
            self.assertTrue(p.configuration_valid(q))
            a = np.array(sender.acceleration())
            self.assertLessEqual(np.max(np.abs(a - previous_a)) / config.period, 4.0 + 1e-6)
            previous_a = a
        self.assertLess(q[2], q0[2] - 0.03)
        for _ in range(1000):
            q = np.array(sender.step(q0))
            self.assertTrue(p.configuration_valid(q))
        np.testing.assert_allclose(q, q0, atol=1e-6)

    def test_stationary_and_nonzero_velocity_stops_use_actual_profile_extrema(self):
        q0 = near_straight("left", 0.3)
        p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), protected_config())
        p.seed(q0)
        sender = p.make_sender_limiter(q0)
        # Ruckig velocity profiles have no target position: an unset pf must
        # never be mistaken for a trajectory passing through the singular zero.
        for _ in range(20):
            np.testing.assert_allclose(sender.step(q0, True), q0, atol=1e-12)
        target = q0.copy()
        target[0] += 0.2
        for _ in range(40):
            q = np.array(sender.follow(target, np.full(6, 0.01), 20.0))
            self.assertTrue(p.configuration_valid(q))
        for _ in range(200):
            q = np.array(sender.step(q, True))
            self.assertTrue(p.configuration_valid(q))
        np.testing.assert_allclose(sender.velocity(), np.zeros(6), atol=1e-10)

    def test_rejected_joint_target_does_not_change_reference_or_sequence(self):
        q0 = near_straight("left")
        p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), protected_config())
        p.seed(q0)
        for axis, value in ((2, -0.1), (4, -0.1), (5, q0[5] + 2 * math.pi)):
            target = q0.copy()
            target[axis] = value
            with self.assertRaises(ValueError):
                p.submit_joints(target, 0.0, 1)
            self.assertEqual(p.status().sequence, 0)
            np.testing.assert_array_equal(p.q(), q0)

    def test_station_sender_never_replaces_collision_approved_sample(self):
        q0 = near_straight("left", 0.06)
        config = protected_config(max_joint_velocity=[0.6] * 6,
                                  max_joint_acceleration=[1.0] * 6,
                                  max_joint_jerk=[4.0] * 6)
        p = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), config)
        p.seed(q0)
        sender = p.make_sender_limiter(q0)
        velocity = np.zeros(6)
        velocity[2] = -0.6
        # Approved q is safe, but its supplied outward velocity cannot stop
        # inside the branch. Substituting another safe q bypasses collision
        # approval, so reject before the native socket write.
        with self.assertRaisesRegex(RuntimeError, "Station-approved"):
            sender.gate(q0, velocity, np.zeros(6), True)
        np.testing.assert_allclose(sender.velocity(), np.zeros(6), atol=1e-12)

    def test_boundary_braking_does_not_ratchet_reference_orientation(self):
        directions = np.random.default_rng(44).normal(size=(30, 3))
        directions /= np.linalg.norm(directions, axis=1)[:, None]
        for index in (8, 15, 21):
            with self.subTest(case=index):
                low = index >= 15
                cfg = protected_config(**(
                    dict(max_joint_velocity=[0.6] * 6, max_joint_acceleration=[1.0] * 6,
                         max_joint_jerk=[4.0] * 6) if low else {}
                ))
                q0 = (near_straight("left") if index == 15
                      else Assembly("diagonal-08").initial_pose("left"))
                sim = Simulation(q=q0, planner_config=cfg)
                home = sim.target.copy()
                target = home.copy()
                target[:3, 3] += 0.3 * directions[index]
                for _ in range(5000):
                    row = sim.step(target)
                    self.assertTrue(sim.planner.configuration_valid(sim.q))
                    reference = sim.model.fk(sim.planner.reference())
                    angle = np.linalg.norm(_native.pose_to_rotvec(
                        (np.linalg.inv(target) @ reference).ravel())[3:])
                    self.assertLessEqual(angle, cfg.boundary_orientation_tolerance + 1e-8)
                self.assertLessEqual(row["orientation_error_rad"],
                                     cfg.boundary_orientation_tolerance + 1e-4)
                run_target(sim, sim.model.fk(sim.q), 1)
                run_target(sim, home, 5000)
                np.testing.assert_allclose(sim.q, q0, atol=1e-3)

    def test_native_feature_is_opt_in_and_invalid_settings_are_rejected(self):
        self.assertFalse(PlannerConfig().preserve_configuration)
        for overrides in (dict(joint_motion_limits=False), dict(configuration_margin=0),
                          dict(configuration_joint_span=math.pi),
                          dict(boundary_orientation_tolerance=float("nan"))):
            with self.assertRaises(ValueError):
                _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(),
                                protected_config(**overrides))


if __name__ == "__main__":
    unittest.main()
