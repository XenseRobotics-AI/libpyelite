"""Joint-current residual estimator (from the compliance WIP) and its contact-monitor wiring."""
import unittest

import numpy as np
from libpyelite import ComplianceConfig, ComplianceController, EliteDriverConfig, load_compliance_config, model_path
from libpyelite.assembly import Assembly
from libpyelite.control_profile import configure_native_driver, contact_monitor_config, native_config

RIGHT = np.array([-1.7453, -2.6180, -1.5708, -2.6180, -1.5708, -0.7854])
DT = 0.004
BIAS = np.array([3.0, -2.0, 1.5, 0.3, -0.2, 0.1])  # constant model error the bias must absorb


def config(**overrides):
    cfg = load_compliance_config(None, gravity=[-6.6, 0.1, -6.3], payload_mass=1.18,
                                 payload_cog=[0.024, 0.0, 0.101], friction_coulomb=[8.6, 8.2, 4.8, 1.3, 1.2, 1.6],
                                 friction_viscous=[14.5, 9.5, 13.0, 0.8, 0.9, 0.0])
    for key, value in overrides.items():
        setattr(cfg, key, value)
    return cfg


class ComplianceTests(unittest.TestCase):
    def setUp(self):
        self.controller = ComplianceController(model_path(), np.eye(4).ravel().tolist(), config())
        self.zero = [0.0] * 6

    def rest_torque(self):
        return np.asarray(self.controller.model_torque(RIGHT.tolist(), self.zero, self.zero)) + BIAS

    def contact_torque(self, force):
        jacobian = np.asarray(self.controller.tcp_jacobian(RIGHT.tolist())).reshape(3, 6)
        return self.rest_torque() - jacobian.T @ np.asarray(force)

    def run_for(self, seconds, tau):
        for _ in range(int(round(seconds / DT))):
            offset = self.controller.update(DT, RIGHT.tolist(), self.zero, np.asarray(tau).tolist())
        return np.asarray(offset)

    def test_bias_then_quiet(self):
        self.run_for(0.2, self.rest_torque())
        self.assertFalse(self.controller.status().biased)
        offset = self.run_for(1.0, self.rest_torque())
        status = self.controller.status()
        self.assertTrue(status.biased)
        np.testing.assert_allclose(status.tau_ext, 0.0, atol=1e-6)
        np.testing.assert_allclose(offset, 0.0, atol=1e-9)
        self.assertFalse(status.in_contact)

    def test_push_moves_target_along_force(self):
        self.run_for(0.5, self.rest_torque())
        applied = np.array([0.0, 0.0, -30.0])
        offset = self.run_for(3.0, self.contact_torque(applied))
        status = self.controller.status()
        force = np.asarray(status.force)
        self.assertTrue(status.in_contact)
        self.assertGreater(force @ applied / (np.linalg.norm(force) * np.linalg.norm(applied)), 0.95)
        self.assertLess(np.linalg.norm(force), 30.0)  # the per-joint bands remove part of it
        self.assertGreater(offset @ applied / (np.linalg.norm(offset) * np.linalg.norm(applied)), 0.95)
        self.assertLessEqual(np.linalg.norm(offset), config().max_offset + 1e-9)
        # Released: the spring brings the target back.
        back = self.run_for(4.0, self.rest_torque())
        self.assertLess(np.linalg.norm(back), 0.1 * np.linalg.norm(offset) + 1e-4)

    def test_small_push_stays_in_deadband(self):
        self.run_for(0.5, self.rest_torque())
        offset = self.run_for(1.0, self.contact_torque([0.0, 2.0, 0.0]))
        np.testing.assert_allclose(offset, 0.0, atol=1e-9)
        self.assertFalse(self.controller.status().in_contact)

    def test_offset_is_bounded(self):
        self.run_for(0.5, self.rest_torque())
        offset = self.run_for(5.0, self.contact_torque([0.0, 55.0, 0.0]))
        self.assertLessEqual(np.linalg.norm(offset), config().max_offset + 1e-9)

    def test_collision_trips_and_latches(self):
        self.run_for(0.5, self.rest_torque())
        tau = self.rest_torque()
        tau[0] += 80.0
        self.run_for(0.3, tau)
        status = self.controller.status()
        self.assertIn("J0", status.collision)
        np.testing.assert_allclose(status.offset, 0.0)
        self.assertEqual(np.asarray(self.run_for(0.1, self.rest_torque())).tolist(), [0.0, 0.0, 0.0])

    def test_disabled_joint_threshold_does_not_trip(self):
        self.controller = ComplianceController(model_path(), np.eye(4).ravel().tolist(),
                                               config(collision_threshold=[0.0] * 6))
        self.run_for(0.5, self.rest_torque())
        tau = self.rest_torque()
        tau[0] += 80.0
        self.run_for(0.3, tau)
        self.assertEqual(self.controller.status().collision, "")

    def test_rebias_restarts_averaging(self):
        self.run_for(0.5, self.rest_torque())
        self.controller.rebias()
        self.assertFalse(self.controller.status().biased)
        self.run_for(0.5, self.rest_torque() + 5.0)
        np.testing.assert_allclose(self.controller.status().tau_ext, 0.0, atol=1e-3)

    def test_disabled_config_never_moves(self):
        controller = ComplianceController(model_path(), np.eye(4).ravel().tolist(), config(enabled=False))
        tau = self.contact_torque([0.0, 0.0, -50.0])
        for _ in range(200):
            offset = controller.update(DT, RIGHT.tolist(), self.zero, tau.tolist())
        np.testing.assert_allclose(offset, 0.0)

    def test_invalid_config_rejected(self):
        with self.assertRaises(ValueError):
            ComplianceController(model_path(), np.eye(4).ravel().tolist(), config(mass=0.0))
        with self.assertRaises(ValueError):
            ComplianceController(model_path(), np.eye(4).ravel().tolist(), config(gravity=[0.0, 0.0, 0.0]))

    def test_load_rejects_unknown_override(self):
        with self.assertRaises(AttributeError):
            load_compliance_config(None, stifness=100.0)
        self.assertTrue(load_compliance_config().enabled)
        self.assertIsInstance(load_compliance_config(), ComplianceConfig)


class ContactMonitorConfigTests(unittest.TestCase):
    def test_disabled_by_default(self):
        cfg = EliteDriverConfig()
        configure_native_driver(cfg, {"type": "bi_elite_cs66_rt"})
        self.assertFalse(cfg.contact_monitor.enabled)
        self.assertIsNone(contact_monitor_config({"native_contact_monitor": {"enabled": False}}))

    def test_gravity_follows_each_arm_mount_and_side_overrides_merge(self):
        assembly = Assembly("diagonal-08")
        settings = {
            "type": "bi_elite_cs66_rt",
            "payload_mass": 0.77,
            "payload_cog": [0.0, 0.0, 0.1],
            "native_contact_monitor": {"collision_threshold": [40.0] * 6},
            "right_native_contact_monitor": {"collision_threshold": [20.0] * 6},
        }
        for side in ("left", "right"):
            cfg = native_config(settings, side, assembly)
            monitor = cfg.contact_monitor
            self.assertTrue(monitor.enabled)
            expected = assembly.world_from_base(side)[:3, :3].T @ [0.0, 0.0, -9.80665]
            np.testing.assert_allclose(monitor.gravity, expected, atol=1e-9)
            self.assertAlmostEqual(monitor.payload_mass, 0.77)
            self.assertEqual(list(monitor.collision_threshold), [40.0 if side == "left" else 20.0] * 6)

    def test_explicit_gravity_wins_and_mountless_arm_needs_it(self):
        monitor = contact_monitor_config({"native_contact_monitor": {"gravity": [-6.6, 0.1, -6.3]}})
        np.testing.assert_allclose(monitor.gravity, [-6.6, 0.1, -6.3])
        with self.assertRaises(ValueError):
            contact_monitor_config({"native_contact_monitor": {"enabled": True}}, side="left")


if __name__ == "__main__":
    unittest.main()
