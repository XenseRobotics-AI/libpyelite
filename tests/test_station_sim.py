"""World/base and controller/assembly mappings used by the dual-arm UI."""
import unittest

import numpy as np
import pinocchio as pin
from libpyelite.sim import StationSimulation


class StationSimulationTests(unittest.TestCase):
    def test_full_station_fk_and_independent_tracking(self):
        sim = StationSimulation()
        self.assertEqual(sim.model.nq, 16)
        self.assertIn("assembly_taccap_identified", str(sim.urdf_path))
        for side in ("left", "right"):
            np.testing.assert_allclose(sim.viewer_q()[sim.indices[side]], sim.arms[side].q, atol=1e-12)
        right = sim.arms["right"].q.copy()
        target = sim.tcp_poses()["left"].copy()
        target[0, 3] += .005
        sim.set_target("left", target)
        for _ in range(200):
            sim.step()
        np.testing.assert_array_equal(sim.arms["right"].q, right)
        np.testing.assert_allclose(sim.tcp_poses()["left"][:3, 3], target[:3, 3], atol=1e-4)
        data = sim.model.createData()
        pin.framesForwardKinematics(sim.model, data, sim.viewer_q())
        for side, pose in sim.tcp_poses().items():
            np.testing.assert_allclose(data.oMf[sim.model.getFrameId(side+"_tool0")].homogeneous @ sim.arms[side].tool,
                                       pose, atol=1e-10)

    def test_changed_target_is_submitted_on_next_tick(self):
        sim = StationSimulation()
        sim.step()
        arm = sim.arms["left"]
        sequence = arm.sequence
        target = sim.targets()["left"].copy()
        target[0, 3] += .005
        sim.set_target("left", target)
        before = arm.q.copy()
        sim.step()  # Only 4 ms since last submit, not the 32-ms heartbeat.
        self.assertEqual(arm.sequence, sequence + 1)
        self.assertGreater(np.linalg.norm(arm.q - before), 0)

    def test_right_reset_preserves_left_target(self):
        sim = StationSimulation()
        target = sim.targets()["left"].copy()
        target[2, 3] += .005
        sim.set_target("left", target)
        q = sim.q.copy()
        q[6] += .1
        sim.reset(q, "right")
        np.testing.assert_allclose(sim.targets()["left"], target, atol=1e-12)
        self.assertAlmostEqual(sim.q[6], q[6])

    def test_target_status_does_not_report_waypoint_success(self):
        import json
        from libpyelite.sim import target_status
        # A 10 m target would trip the packaged profile's jump guard; this test is about the status labels.
        settings = json.loads((StationSimulation().assembly.root / "simulation_profile.json").read_text())
        sim = StationSimulation(settings=settings | {"native_max_target_translation_jump": 0.0})
        target = sim.targets()["left"].copy()
        target[0, 3] += 10
        sim.set_target("left", target)
        sample = sim.step()["left"]
        self.assertFalse(sample["qp_failed"])
        # The waypoint solve succeeded, but the user target is 10 m away.
        self.assertEqual(target_status(sample)[0], "tracking")
        self.assertEqual(target_status(dict(sample, success=False, progress=False))[0], "error")
        sample["qp_failed"] = True
        level, message = target_status(sample)
        self.assertEqual(level, "error")
        self.assertIn("QP", message)

    def test_zero_iteration_waypoint_solve_is_not_reported_as_failure(self):
        from libpyelite.sim import target_status
        sample = {"qp_failed": False, "success": True, "progress": False,
                  "position_error_m": 0.05, "orientation_error_rad": 0.0}
        self.assertEqual(target_status(sample)[0], "tracking")
        self.assertEqual(target_status(dict(sample, position_error_m=5e-5))[0], "ok")
