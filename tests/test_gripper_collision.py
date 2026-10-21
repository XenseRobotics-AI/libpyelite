"""Measured gripper geometry and timestamp validation; never opens hardware."""

import hashlib
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from libpyelite.collision import StationLayer
from libpyelite import PinkIKOptions, PlannerConfig, _native, model_path


class GripperCollisionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        # Twelve scalar arm coordinates plus two opposing fingers on each side.
        # Finger prismatic coordinates make the geometric effect easy to inspect.
        parts = ['<robot name="test"><link name="world"/>']
        for side, x in (("left", -1), ("right", 1)):
            parent = "world"
            for i, joint in enumerate(StationLayer.JOINTS):
                child = side + ("_tool0" if i == 5 else f"_link{i}")
                parts.append(
                    f'<link name="{child}"/><joint name="{side}_{joint}" type="revolute">'
                    f'<parent link="{parent}"/><child link="{child}"/>'
                    f'<origin xyz="{x if i == 0 else 0} 0 0"/><axis xyz="0 0 1"/>'
                    '<limit lower="-3" upper="3" effort="1" velocity="1"/></joint>'
                )
                parent = child
            for name, low, high, mimic in (
                ("driver", 0, 0.5, ""),
                ("follower", -0.5, 0, f'<mimic joint="{side}_driver" multiplier="-1" offset="0"/>'),
            ):
                parts.append(
                    f'<link name="{side}_{name}_link"/><joint name="{side}_{name}" type="prismatic">'
                    f'<parent link="{parent}"/><child link="{side}_{name}_link"/><axis xyz="1 0 0"/>'
                    f'<limit lower="{low}" upper="{high}" effort="1" velocity="1"/>{mimic}</joint>'
                )
        parts.append("</robot>")
        urdf = root / "station.urdf"
        urdf.write_text("".join(parts))
        profile = {
            "schema": 1, "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
            "margin_m": 0.02, "detection_distance_m": 1.0, "gain": 0.2, "allowed_link_pairs": [],
            "shapes": [
                {"name": "finger", "frame": "left_driver_link", "group": "left",
                 "type": "box", "size": [0.1] * 3, "xyz": [0, 0, 0], "rpy": [0, 0, 0]},
                {"name": "wall", "frame": "world", "group": "environment",
                 "type": "box", "size": [0.1] * 3, "xyz": [-0.45, 0, 0], "rpy": [0, 0, 0]},
            ],
        }
        (root / "collision.json").write_text(json.dumps(profile))
        assembly = SimpleNamespace(root=root, urdf=urdf, viewer_joint_defaults=lambda: {})
        self.layer = StationLayer(assembly, "auto", [np.eye(4), np.eye(4)])
        self.calibration = {
            side: {side + "_driver": [[0, 0], [0.5, 0.1], [1, 0.5]]} for side in ("left", "right")
        }
        self.q = [0.0] * 6

    def configure(self, age=0.1):
        self.layer.configure_grippers(self.calibration, age)

    def publish(self, opening=0.5, *, age=0.0, seq=1, valid=True, side=None, stamp=None):
        stamp = time.monotonic() - age if stamp is None else stamp
        samples = {
            name: SimpleNamespace(position=opening, sampled_at=stamp, sequence=seq, valid=valid)
            for name in ("left", "right")
        }
        if side is not None:
            samples[side].valid = False
        self.layer.publish_gripper_snapshots(samples)

    def station_q(self):
        return self.layer.station_q(self.q, self.q)

    def test_uncalibrated_simulation_retains_static_defaults(self):
        np.testing.assert_array_equal(self.station_q(), np.zeros(self.layer.model.nq))

    def test_exact_endpoints_intermediate_knots_and_mimics(self):
        self.configure()
        for seq, (opening, expected) in enumerate(((0, 0), (0.25, 0.05), (0.5, 0.1), (0.75, 0.3), (1, 0.5)), 1):
            self.publish(opening, seq=seq)
            q = self.station_q()
            for side in ("left", "right"):
                for suffix, value in (("driver", expected), ("follower", -expected)):
                    joint = self.layer.model.joints[self.layer.model.getJointId(side + "_" + suffix)]
                    self.assertAlmostEqual(q[joint.idx_q], value)

    def test_measurements_update_collision_while_arm_joints_hold(self):
        self.configure()
        self.publish(0, seq=1)
        closed = self.station_q()
        before = self.layer.collision.check(closed)
        self.publish(1, seq=2)
        opened = self.station_q()
        after = self.layer.collision.check(opened)
        self.assertTrue(before.safe)
        self.assertFalse(after.safe)
        self.assertGreater(before.minimum_distance, after.minimum_distance)
        for indices in self.layer.indices.values():
            np.testing.assert_array_equal(closed[indices], opened[indices])

    def test_live_gripper_margin_and_overlap_hold_then_resume_both_planner_modes(self):
        self.configure()
        seq = 0
        for limited in (False, True):
            with self.subTest(joint_motion_limits=limited):
                config = PlannerConfig()
                config.joint_motion_limits = limited
                config.max_joint_velocity = [0.2] * 6
                config.max_joint_acceleration = [0.5] * 6
                config.max_joint_jerk = [10.0] * 6
                planner = _native.Planner(model_path(), np.eye(4).ravel(), PinkIKOptions(), config)
                planner.seed(self.q)
                now = time.monotonic()
                planner.submit_joints([0.05, 0, 0, 0, 0, 0], now, 1)
                seq += 1
                self.publish(0, seq=seq)
                result = self.layer.guard.tick(planner, None, self.q, self.q, now)
                previous = np.asarray(result.q[0]).copy()
                reference = np.asarray(planner.reference()).copy()
                for opening in (0.91875, 1.0, 0.99, 1.0):
                    seq += 1
                    self.publish(opening, seq=seq)
                    result = self.layer.guard.tick(planner, None, self.q, self.q, now + 0.004)
                    self.assertTrue(result.measured_geometry_hold)
                    self.assertTrue(result.held[0])
                    self.assertFalse(result.clearance.safe)
                    if opening == 0.91875:
                        self.assertGreater(result.clearance.minimum_distance, 0)  # only margin violated
                    if opening == 1:
                        self.assertLess(result.clearance.minimum_distance, 0)  # actual overlap
                    np.testing.assert_array_equal(result.q[0], previous)
                    np.testing.assert_array_equal(planner.reference(), reference)
                seq += 1
                self.publish(0, seq=seq)
                result = self.layer.guard.tick(planner, None, self.q, self.q, now + 0.008)
                self.assertFalse(result.measured_geometry_hold)
                self.assertGreater(np.linalg.norm(np.asarray(result.q[0]) - previous), 0)
                seq += 1
                self.publish(0, seq=seq, age=1)
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    self.layer.guard.tick(planner, None, self.q, self.q, now + 0.012)

    def test_first_snapshot_is_required_even_when_no_arm_is_running(self):
        self.configure()
        with self.assertRaisesRegex(RuntimeError, "uninitialized"):
            self.layer.guard.tick(None, None, self.q, self.q, 0)

    def test_stale_future_nan_and_invalid_data_fail_closed(self):
        self.configure()
        for seq, (kwargs, reason) in enumerate(
            (({"age": 0.2}, "Left.*is stale"), ({"age": -1.0}, "Left.*is from the future"),
             ({"stamp": float("nan")}, "Left.*invalid timestamp"), ({"seq": 0}, "Left.*is invalid"),
             ({"valid": False}, "Left.*is invalid"), ({"side": "right"}, "Right.*is invalid"),
             ({"opening": float("nan")}, "Left.*is invalid"), ({"opening": 1.001}, "Left.*is invalid")), 1
        ):
            with self.subTest(kwargs=kwargs):
                # Independent guards avoid a deliberately invalid future timestamp
                # changing this test's sequence watermark.
                self.layer.guard = type(self.layer.guard)(
                    self.layer.collision.native, str(self.layer.urdf), self.layer.defaults,
                    [np.eye(4).ravel().tolist()] * 2,
                )
                maps = self.layer._gripper_maps
                self.layer.guard.configure_grippers([m[0] for m in maps], [m[1] for m in maps], 0.1)
                self.publish(**({"seq": seq} | kwargs))
                with self.assertRaisesRegex(RuntimeError, reason) as failure:
                    self.station_q()
                self.assertIn("sequence=", str(failure.exception))
                self.assertIn("age_s=", str(failure.exception))
                self.assertIn("max_age_s=0.1", str(failure.exception))

    def test_same_sequence_never_refreshes_timestamp_even_across_invalid_state(self):
        self.configure()
        self.publish(age=0.2, seq=10)
        self.layer.publish_gripper_snapshots({})
        self.publish(age=0, seq=10)
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.station_q()
        self.publish(seq=11)
        self.station_q()

    def test_same_sequence_with_changed_position_and_regressed_sequence_are_invalid(self):
        self.configure()
        self.publish(0.2, seq=10)
        self.publish(0.8, seq=10)
        with self.assertRaises(RuntimeError):
            self.station_q()
        self.publish(0.2, seq=9)
        with self.assertRaises(RuntimeError):
            self.station_q()

    def test_freshness_uses_read_time_not_old_planner_logical_time(self):
        self.configure()
        self.publish()
        result = self.layer.guard.tick(None, None, self.q, self.q, 0.0)
        self.assertTrue(all(0 <= age < 0.1 for age in result.gripper_age_s))
        self.assertEqual(list(result.gripper_sequence), [1, 1])

    def test_concurrent_publication_is_coherent_and_not_spuriously_future(self):
        self.configure(age=1.0)
        self.publish()
        stop = threading.Event()
        def producer():
            seq = 2
            while not stop.is_set():
                self.publish((seq % 2), seq=seq)
                seq += 1
                stop.wait(0.0005)
        thread = threading.Thread(target=producer)
        thread.start()
        try:
            for _ in range(100):
                # Native checks time after its snapshot load, not this deliberately old tick time.
                result = self.layer.guard.tick(None, None, self.q, self.q, 0.0)
                self.assertEqual(result.gripper_sequence[0], result.gripper_sequence[1])
                q = self.station_q()
                values = [q[self.layer.model.joints[self.layer.model.getJointId(s + "_driver")].idx_q]
                          for s in ("left", "right")]
                self.assertEqual(values[0], values[1])
        finally:
            stop.set()
            thread.join()

    def test_missing_invalid_and_incomplete_calibrations_are_rejected(self):
        for calibration in (None, {}, {"left": self.calibration["left"]},
                            {"left": {"left_driver": [[0, 0], [0, 0.5]]}},
                            {"left": {"left_driver": [[0, 0], [1, 1]]}}):
            with self.subTest(calibration=calibration), self.assertRaises(ValueError):
                self.layer.configure_grippers(calibration)
        self.configure()  # failed validation has not partially configured the guard

    def test_mapping_cannot_overwrite_an_arm_joint(self):
        with self.assertRaisesRegex(ValueError, "arm joint"):
            self.layer.guard.configure_grippers(["left_shoulder_pan_joint"], [0], 0.1)

    def test_preflight_owns_independent_collision_caches(self):
        self.configure()
        self.publish()
        before = self.layer.collision._linearized
        joints = {"left": self.q, "right": self.q}
        self.layer.check_joint_path(joints, joints)
        self.assertIsNot(self.layer._path_collision.native, self.layer.collision.native)
        self.assertIs(self.layer.collision._linearized, before)


if __name__ == "__main__":
    unittest.main()
