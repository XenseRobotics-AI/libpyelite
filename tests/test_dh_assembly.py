"""Bench 08 DH geometry, joint representation and native collision behavior."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from libpyelite.assembly import Assembly, available_models
from libpyelite.collision import StationCollision, StationLayer
from libpyelite.sim import StationSimulation


def expanded_q(model, tree, arm_positions, closing):
    joints = {j.get('name'): j for j in tree.findall('joint')}
    angles = {side + '_tool_gripper_finger1_joint': closing[i] for i, side in enumerate(('left', 'right'))}
    angles.update(arm_positions)
    def angle(name):
        if name not in angles:
            mimic = joints[name].find('mimic')
            angles[name] = angle(mimic.get('joint')) * float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0))
        return angles[name]
    q = pin.neutral(model)
    for name, element in joints.items():
        if element.get('type') == 'fixed':
            continue
        joint = model.joints[model.getJointId(name)]
        value = angle(name)
        q[joint.idx_q:joint.idx_q + joint.nq] = [np.cos(value), np.sin(value)] if joint.nq == 2 else [value]
    return q


class DhAssemblyTests(unittest.TestCase):
    def test_model_selection_preserves_taccap_defaults(self):
        self.assertEqual(Assembly('diagonal-08').name, 'diagonal-08-taccap')
        self.assertEqual(Assembly('diagonal-08-taccap').root, Assembly('diagonal-08').root)
        self.assertEqual(Assembly('diagonal-08', end_effector='dh-ag95').name, 'diagonal-08-dh-ag95')
        self.assertIn('diagonal-08-dh-ag95', [m.id for m in available_models()])
        with self.assertRaises(ValueError):
            Assembly('diagonal-07', end_effector='dh-ag95')
        with self.assertRaisesRegex(ValueError, 'must agree'):
            StationSimulation('diagonal-08-dh-ag95', settings={'native_assembly': 'diagonal-08'})
        sim = StationSimulation(settings={'native_assembly': 'diagonal-08-dh-ag95'})
        self.assertEqual(sim.assembly.end_effector, 'dh-ag95')

    def test_installed_snapshot_resolves_and_hashes_match(self):
        assembly = Assembly('diagonal-08-dh-ag95')
        manifest = json.loads((assembly.root / 'manifest.json').read_text())
        for name, expected in manifest['files_sha256'].items():
            self.assertEqual(hashlib.sha256((assembly.root / name).read_bytes()).hexdigest(), expected, name)
        self.assertEqual(hashlib.sha256(assembly.urdf.read_bytes()).hexdigest(), assembly.metadata['assembly_sha256'])
        for mesh in ET.parse(assembly.urdf).getroot().iter('mesh'):
            path = (assembly.urdf.parent / mesh.get('filename')).resolve()
            self.assertTrue(path.is_relative_to(assembly.root.resolve()))
            self.assertTrue(path.is_file(), str(path))
        sim = StationSimulation(assembly.name, collision_profile='auto')
        self.assertIsNone(sim.collision_error)
        self.assertEqual(sim.model.nq, 28)
        self.assertEqual(sim.model.nv, 28)
        for arm in sim.arms.values():
            np.testing.assert_array_equal(arm.tool, np.eye(4))
        self.assertTrue(sim.settings['simulation_only'])

    def test_scalar_followers_preserve_all_source_frames_over_opening_range(self):
        assembly = Assembly('diagonal-08-dh-ag95')
        source = assembly.root / assembly.metadata['source_assembly_urdf']
        trees = [ET.parse(p).getroot() for p in (source, assembly.urdf)]
        models = [pin.buildModelFromUrdf(str(p)) for p in (source, assembly.urdf)]
        self.assertEqual(models[0].nq, 40)
        data = [m.createData() for m in models]
        rng = np.random.default_rng(18)
        for closing in np.linspace(0, 0.6524, 21):
            angles = {side + '_' + name: float(value) for side in ('left', 'right')
                      for name, value in zip(StationLayer.JOINTS, assembly.initial_pose(side) + rng.uniform(-.15, .15, 6))}
            for m, d, tree in zip(models, data, trees):
                pin.framesForwardKinematics(m, d, expanded_q(m, tree, angles, [closing, .6524 - closing]))
            for frame in models[0].frames:
                np.testing.assert_allclose(data[0].oMf[models[0].getFrameId(frame.name, frame.type)].homogeneous,
                                           data[1].oMf[models[1].getFrameId(frame.name, frame.type)].homogeneous, atol=1e-12)

    def test_tool_convexes_are_bound_without_refitting_or_double_rotation(self):
        assembly = Assembly('diagonal-08-dh-ag95')
        source = json.loads((assembly.root / 'source_assets/grippers/dh-robotics/ag95/collision/default.json').read_text())
        sim = StationSimulation(assembly.name, collision_profile='auto')
        shapes = {s['name']: s for s in sim.collision.shapes}
        for side in ('left', 'right'):
            for original in source['shapes']:
                actual = shapes[side + '_tool_' + original['name']]
                for field in ('vertices', 'faces', 'xyz', 'rpy'):
                    self.assertEqual(actual[field], original[field])
                self.assertEqual(actual['group'], side)
        self.assertEqual(len(shapes), 345)

    def test_opening_feedback_expands_all_mimics_and_native_guard_accepts_it(self):
        sim = StationSimulation('diagonal-08-dh-ag95', collision_profile='auto')
        layer = StationLayer(sim.assembly, 'auto', [arm.tool for arm in sim.arms.values()])
        layer.configure_grippers(sim.settings['native_collision_gripper_calibration'])
        for seq, opening in enumerate((1., .5, 0.), 1):
            stamp = time.monotonic()
            layer.publish_gripper_snapshots({side: SimpleNamespace(position=opening, sampled_at=stamp, sequence=seq, valid=True)
                                             for side in sim.arms})
            q = layer.station_q(sim.arms['left'].q, sim.arms['right'].q)
            for side in sim.arms:
                driver = layer.model.joints[layer.model.getJointId(side + '_tool_gripper_finger1_joint')]
                tip = layer.model.joints[layer.model.getJointId(side + '_tool_gripper_finger1_finger_tip_joint')]
                self.assertAlmostEqual(q[driver.idx_q], (1 - opening) * .6524)
                self.assertAlmostEqual(q[tip.idx_q], (1 - opening) * .6524 * 1.49462955)

    def test_fingertip_exception_is_local_and_external_obstacles_still_block(self):
        sim = StationSimulation('diagonal-08-dh-ag95', collision_profile='auto')
        pairs = {frozenset((sim.collision.shapes[i]['frame'], sim.collision.shapes[j]['frame'])) for i, j in sim.collision.pairs}
        left = 'left_tool_gripper_finger1_finger_tip_link'
        right = 'right_tool_gripper_finger1_finger_tip_link'
        self.assertNotIn(frozenset((left, 'left_tool_gripper_finger2_finger_tip_link')), pairs)
        for other in (right, 'worktable_120cm_base', 'elite_cs66_a1_stand_base', 'left_forearm_link', 'right_forearm_link'):
            self.assertIn(frozenset((left, other)), pairs)
        q = sim.viewer_q()
        d = sim.model.createData()
        pin.framesForwardKinematics(sim.model, d, q)
        shape = next(s for s in sim.collision.shapes if s['frame'] == left)
        position = d.oMf[sim.model.getFrameId(left)].act(np.mean(shape['vertices'], axis=0))
        profile = copy.deepcopy(sim.collision.profile)
        profile['shapes'].append({'name': 'probe', 'frame': 'world', 'group': 'environment', 'type': 'box',
                                  'size': [.01] * 3, 'xyz': position.tolist(), 'rpy': [0, 0, 0]})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'collision.json'
            path.write_text(json.dumps(profile))
            probe = StationCollision(sim.model, path, sim.urdf_path)
            result = probe.check(q)
            self.assertFalse(result.safe)
            self.assertIn('probe', result.pair)
        target = sim.tcp_poses()['left'].copy()
        target[0, 3] += .005
        sim.set_target('left', target)
        for _ in range(120):
            sim.step()
        self.assertLess(np.linalg.norm(sim.tcp_poses()['left'][:3, 3] - target[:3, 3]), .001)

    def test_full_opening_range_and_certificate_assumptions(self):
        sim = StationSimulation('diagonal-08-dh-ag95', collision_profile='auto')
        tree = ET.parse(sim.urdf_path).getroot()
        angles = {side + '_' + name: value for side, arm in sim.arms.items()
                  for name, value in zip(sim.joint_names, arm.q)}
        previous = None
        for value in np.linspace(0, .6524, 101):
            q = expanded_q(sim.model, tree, angles, [value, .6524 - value])
            self.assertTrue(sim.collision.check(q).safe)
            if previous is not None:
                self.assertTrue(sim.collision.check_segment(previous, q).safe)
            previous = q
        bad = sim.viewer_q()
        joint = sim.model.joints[sim.model.getJointId('left_tool_gripper_finger2_inner_knuckle_joint')]
        bad[joint.idx_q] = .1
        with self.assertRaisesRegex(ValueError, 'mimic'):
            sim.collision.check(bad)
        profile = copy.deepcopy(sim.collision.profile)
        profile['self_margin_m'] = .001
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'collision.json'
            path.write_text(json.dumps(profile))
            with self.assertRaisesRegex(ValueError, 'certificate'):
                StationCollision(sim.model, path, sim.urdf_path)

    def test_separation_proof_rejects_the_actual_fingertip_overlap(self):
        path = Path(__file__).resolve().parents[1] / 'scripts/prove_dh_separation.py'
        spec = importlib.util.spec_from_file_location('dh_separation', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        assembly = Assembly('diagonal-08-dh-ag95')
        source = assembly.root / 'source_assets/grippers/dh-robotics/ag95/urdf/ag95.urdf'
        with self.assertRaisesRegex(ValueError, 'Cannot certify'):
            module.prove_separation(source, [['gripper_finger1_finger_tip_link', 'gripper_finger2_finger_tip_link']], .0005)


if __name__ == '__main__':
    unittest.main()
