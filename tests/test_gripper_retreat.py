"""Requested retreat after measured fingers enter the table clearance margin."""

import time
import unittest
from types import SimpleNamespace

import numpy as np

from libpyelite.collision import StationLayer
from libpyelite.sim import StationSimulation


class GripperRetreatTests(unittest.TestCase):
    JOINT_LIMITED = True
    # Last safe command before the Nov 2 14:42:25 closure, reconstructed from
    # the logged TCP stream with the production planner and diagonal-08 model.
    START = [
        -1.4798167922245027, -0.3836239087123868, 1.591963640233949,
        -0.39422574327283844, 1.3681539874528523, -0.5533398209284142,
        -1.7455834412046782, -2.6244852259816494, -1.561326580852727,
        -2.6209665685533454, -1.5715123894554412, -0.7854770409122054,
    ]

    def setUp(self):
        self.sim = StationSimulation(initial_q=self.START, collision_profile="auto")
        if not self.JOINT_LIMITED:
            self.sim = StationSimulation(initial_q=self.START, collision_profile="auto", settings={
                **self.sim.settings, "native_joint_motion_limits": False, "native_preserve_configuration": False,
            })
        self.layer = StationLayer(self.sim.assembly, "auto", [a.tool for a in self.sim.arms.values()])
        self.layer.configure_grippers(self.sim.settings["native_collision_gripper_calibration"])
        self.sim.guard = self.layer.guard
        self.sim.collision = self.layer.collision
        self.sequence = 0
        self.opening = 1.0
        self.home = self.sim.tcp_poses()["left"].copy()

    def publish(self):
        self.sequence += 1
        self.layer.publish_gripper_snapshots({
            side: SimpleNamespace(position=self.opening if side == "left" else 1.0,
                                  sampled_at=time.monotonic(), sequence=self.sequence, valid=True)
            for side in self.sim.arms
        })

    def step(self):
        self.publish()
        sample = self.sim.step()
        for arm in self.sim.arms.values():
            # NativeDriver publishes planner.q(), not GuardResult.q.
            np.testing.assert_array_equal(arm.planner.q(), arm.q)
        return sample

    def clearance(self):
        self.publish()
        return self.layer.collision.check(self.layer.station_q(
            self.sim.arms["left"].q, self.sim.arms["right"].q))

    def close(self):
        self.assertTrue(self.clearance().safe)
        for self.opening in np.linspace(1.0, 0.0, 26):
            self.step()
        closed = self.clearance()
        self.assertFalse(closed.safe)
        self.assertGreater(closed.minimum_distance, 0)

    def test_closed_gripper_can_follow_upward_retreat_before_reopening(self):
        self.close()
        target = self.home.copy()
        target[2, 3] += 0.08
        self.sim.set_target("left", target)
        right = self.sim.arms["right"].q.copy()
        previous_distance = self.clearance().minimum_distance
        recovering = True
        for _ in range(500):
            self.step()
            current = self.clearance()
            if recovering:
                self.assertGreaterEqual(current.minimum_distance, previous_distance - 1e-9)
                previous_distance = current.minimum_distance
                recovering = not current.safe
        np.testing.assert_allclose(self.sim.tcp_poses()["left"], target, atol=1e-4)
        np.testing.assert_array_equal(self.sim.arms["right"].q, right)
        self.assertTrue(self.clearance().safe)

        held = self.sim.tcp_poses()["left"].copy()
        self.opening = 1.0
        for _ in range(250):
            self.step()
        np.testing.assert_allclose(self.sim.tcp_poses()["left"], held, atol=1e-5)

    def test_closing_does_not_create_an_automatic_escape_motion(self):
        self.close()
        before = self.sim.q.copy()
        for _ in range(500):
            self.step()
        np.testing.assert_allclose(self.sim.q, before, atol=1e-10)

    def test_joint_target_can_retreat_with_the_gripper_closed(self):
        self.close()
        arm = self.sim.arms["left"]
        target = self.home.copy()
        target[2, 3] += 0.02
        base_target = np.linalg.inv(self.sim.assembly.world_from_base("left")) @ target
        joints, diagnostics = arm.model.solve(base_target, arm.q)
        self.assertTrue(diagnostics.success)
        self.sim.set_joint_target("left", joints)
        for _ in range(500):
            self.step()
        np.testing.assert_allclose(self.sim.arms["left"].q, joints, atol=1e-5)
        self.assertTrue(self.clearance().safe)

    def test_downward_target_does_not_deepen_closed_gripper_clearance_violation(self):
        self.close()
        before = self.clearance().minimum_distance
        target = self.home.copy()
        target[2, 3] -= 0.05
        self.sim.set_target("left", target)
        for _ in range(250):
            self.step()
            self.assertGreaterEqual(self.clearance().minimum_distance, before - 1e-9)
        np.testing.assert_allclose(self.sim.tcp_poses()["left"], self.home, atol=1e-5)


class GripperRetreatWithoutJointLimiterTests(GripperRetreatTests):
    JOINT_LIMITED = False


class MovingGripperClosureTests(unittest.TestCase):
    def test_lift_after_closing_during_a_moving_approach(self):
        from pathlib import Path
        import pinocchio as pin

        recording = np.load(Path(__file__).with_name('fixtures') / 'gripper_close_lift_20261103.npz')
        events = recording['events']
        sim = StationSimulation(collision_profile='auto')
        layer = StationLayer(sim.assembly, 'auto', [a.tool for a in sim.arms.values()])
        layer.configure_grippers(sim.settings['native_collision_gripper_calibration'])
        sim.guard, sim.collision = layer.guard, layer.collision
        opening = [1., 1.]
        index = 0
        checkpoint = 0
        pauses = []
        pause = 0
        for tick in range(int(np.ceil(float(recording['duration']) / .004)) + 1):
            now = tick * .004
            while index < len(events) and events[index, 0] <= now:
                event = events[index]
                pose = np.eye(4)
                pose[:3, 3] = event[2:5]
                pose[:3, :3] = pin.exp3(event[5:8])
                sim.arms[('left', 'right')[int(event[1])]].set_target(pose)
                opening = event[8:10]
                index += 1
            layer.publish_gripper_snapshots({
                side: SimpleNamespace(position=float(opening[i]), sampled_at=time.monotonic(),
                                      sequence=tick + 1, valid=True)
                for i, side in enumerate(sim.arms)
            })
            previous = sim.arms['left'].q.copy()
            sim.step()
            target = sim.targets()['left']
            actual = sim.tcp_poses()['left']
            if opening[0] == 0 and target[2, 3] - actual[2, 3] > .005:
                pause = pause + 1 if np.max(np.abs(sim.arms['left'].q - previous)) < 1e-9 else 0
                pauses.append(pause)
            else:
                pause = 0
            if checkpoint < len(recording['checkpoints']) and now >= recording['checkpoints'][checkpoint]:
                with self.subTest(clock=float(recording['clock_start']) + now):
                    self.assertEqual(opening[0], 0.)
                    self.assertLess(np.linalg.norm(actual[:3, 3] - target[:3, 3]), .01)
                checkpoint += 1
        self.assertEqual(checkpoint, len(recording['checkpoints']))
        self.assertLessEqual(max(pauses, default=0) * .004, .02)

