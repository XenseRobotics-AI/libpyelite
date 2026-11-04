"""Import the DH tool into bench 08 using the packaged arm/environment envelopes.

Run into an empty destination, or use --check to verify the packaged snapshot.
No robot connection, payload identification or hardware calibration is performed.
"""
import argparse
import copy
import itertools
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from import_assemblies import derive_chain, digest, fixed_transform
from prove_dh_separation import prove_separation

STATIONS = Path(__file__).resolve().parents[1] / 'libpyelite/assets/stations'
POLICY = Path(__file__).with_name('dh_ag95_collision_policy.json')
MODEL_ID = 'diagonal-08-dh-ag95'
SIDES = ('left', 'right')


def is_tool(name):
    return name.startswith(('left_tool_', 'right_tool_', 'left_adapter_', 'right_adapter_'))


def geometry_signature(urdf):
    root = ET.parse(urdf).getroot()
    result = {}
    for link in root.findall('link'):
        name = link.get('name')
        if is_tool(name):
            continue
        tag = 'collision' if name.startswith(('left_', 'right_')) else 'visual'
        shapes = []
        for element in link.findall(tag):
            node = copy.deepcopy(element)
            for mesh in node.iter('mesh'):
                mesh.set('filename', digest((urdf.parent / mesh.get('filename')).resolve()))
            shapes.append(ET.tostring(node, encoding='unicode').strip())
        result[name] = shapes
    result['joints'] = {
        j.get('name'): ET.tostring(j, encoding='unicode').strip()
        for j in root.findall('joint')
        if not any(is_tool(j.find(tag).get('link')) for tag in ('parent', 'child'))
    }
    return result


def scalarize_mimics(root):
    """Preserve q/axes/origins; bound continuous followers by their driver range."""
    joints = {j.get('name'): j for j in root.findall('joint')}
    converted = []
    def limits(name, active=()):
        if name in active:
            raise ValueError('Cyclic gripper mimic')
        joint = joints[name]
        mimic = joint.find('mimic')
        if mimic is None:
            limit = joint.find('limit')
            return np.array([float(limit.get('lower')), float(limit.get('upper'))])
        bounds = limits(mimic.get('joint'), (*active, name))
        return np.sort(bounds * float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0)))
    for name, joint in joints.items():
        if joint.get('type') != 'continuous':
            continue
        if not is_tool(name) or joint.find('mimic') is None:
            raise ValueError(f'Only tool continuous mimic joints may be converted: {name}')
        lower, upper = limits(name)
        if not np.isfinite([lower, upper]).all() or lower >= upper:
            raise ValueError(f'Invalid derived mimic limits: {name}')
        joint.set('type', 'revolute')
        joint.find('limit').set('lower', format(lower, '.17g'))
        joint.find('limit').set('upper', format(upper, '.17g'))
        converted.append({'joint': name, 'lower': float(lower), 'upper': float(upper)})
    return converted


def import_station(source, destination):
    source, destination = source.resolve(), destination.resolve()
    if destination.exists():
        raise FileExistsError(f'Refusing to overwrite {destination}')
    assembly_dir = source / 'assets/assemblies/diagonal-07-elite-cs66'
    record_path = assembly_dir / 'assembly.dh-ag95.json'
    record = json.loads(record_path.read_text())
    urdf = assembly_dir / record['entrypoint']
    baseline = STATIONS / 'diagonal-08'
    baseline_meta = json.loads((baseline / 'control.json').read_text())
    baseline_urdf = baseline / baseline_meta['assembly_urdf']
    if geometry_signature(urdf) != geometry_signature(baseline_urdf):
        raise ValueError('Bench 08 arm/environment geometry or joint frames changed; review envelopes before importing')
    tool = source / 'assets/grippers/dh-robotics/ag95'
    tool_profile_path = tool / 'collision/default.json'
    tool_profile = json.loads(tool_profile_path.read_text())
    policy = json.loads(POLICY.read_text())
    if (digest(tool / 'urdf/ag95.urdf') != policy['tool_urdf_sha256'] or
            digest(tool_profile_path) != policy['tool_profile_sha256']):
        raise ValueError('DH geometry changed: review dh_ag95_collision_policy.json, including temporary fingertip exclusions')
    if tool_profile['urdf_sha256'] != policy['tool_urdf_sha256']:
        raise ValueError('DH component profile is stale')
    root = ET.parse(urdf).getroot()
    asset_index = {d['id']: (p.parent, d) for p in (source / 'assets').rglob('asset.json')
                   for d in [json.loads(p.read_text())]}
    for instance, spec in record['instances'].items():
        asset, data = asset_index[spec['asset_id']]
        entry = data['entrypoints'][spec['variant']]
        entry = entry['urdf'] if isinstance(entry, dict) else entry
        if digest(asset / entry) != record['inputs'][instance]:
            raise ValueError(f'Assembly input hash drift: {instance}')
    destination.mkdir(parents=True)
    files = {}
    def snapshot(path):
        path = path.resolve()
        relative = Path('source_assets') / path.relative_to(source / 'assets')
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        files[str(relative)] = digest(path)
        return relative
    for path in (record_path, urdf, assembly_dir / 'config/start_pose.dh-ag95.json',
                 assembly_dir / 'README.dh-ag95.md', assembly_dir / 'COMPONENTS.dh-ag95.md'):
        snapshot(path)
    for mesh in root.iter('mesh'):
        snapshot(urdf.parent / mesh.get('filename'))
    for spec in record['instances'].values():
        asset, data = asset_index[spec['asset_id']]
        entry = data['entrypoints'][spec['variant']]
        snapshot(asset / (entry['urdf'] if isinstance(entry, dict) else entry))
        for name in ('asset.json', 'README.md', 'NOTICE.md'):
            if (asset / name).exists():
                snapshot(asset / name)
        for path in asset.rglob('*'):
            if path.is_file() and ('LICENSE' in str(path.relative_to(asset))):
                snapshot(path)
    for path in (tool_profile_path, tool / 'collision/recipe.json', tool / 'provenance/collision-derivation.json',
                 tool / 'provenance/urdf/ag95.urdf', tool / 'provenance/urdf/dh_robotics_ag95.urdf',
                 tool / 'provenance/package.xml.txt'):
        snapshot(path)
    shutil.copyfile(POLICY, destination / 'collision_policy.json')
    files['collision_policy.json'] = digest(POLICY)
    (destination / 'provenance').mkdir()
    for name in ('control.json', 'collision.json', 'simulation_profile.json'):
        relative = 'provenance/bench08_' + name
        shutil.copyfile(baseline / name, destination / relative)
        files[relative] = digest(baseline / name)
    source_urdf = snapshot(urdf)
    converted = scalarize_mimics(root)
    root.set('name', MODEL_ID)
    for mesh in root.iter('mesh'):
        path = (urdf.parent / mesh.get('filename')).resolve()
        mesh.set('filename', str(Path('source_assets') / path.relative_to(source / 'assets')))
    derived = destination / 'assembly_dh_ag95.urdf'
    ET.indent(root)
    ET.ElementTree(root).write(derived, encoding='utf-8', xml_declaration=True)
    metadata = {
        'assembly_id': MODEL_ID, 'source_assembly_id': record['id'], 'station': 'diagonal-08',
        'end_effector': 'dh-ag95', 'assembly_urdf': derived.name, 'assembly_sha256': digest(derived),
        'source_assembly_urdf': str(source_urdf), 'source_assembly_sha256': digest(urdf),
        'joint_mapping': 'q_assembly = q_controller; DH angles and mimic coefficients retain source convention',
        'continuous_mimic_derivation': converted,
        'viewer_joint_defaults': {j.get('name'): 0.0 for j in root.findall('joint')
                                  if is_tool(j.get('name')) and j.get('type') != 'fixed'},
        'arms': {}, 'simulation_only': True,
        'tcp_reference': 'tool0 flange; DH TCP calibration not supplied',
    }
    for side in SIDES:
        chain = derive_chain(root, side, [0.] * 6)
        path = destination / f'{side}_control.urdf'
        ET.indent(chain)
        ET.ElementTree(chain).write(path, encoding='utf-8', xml_declaration=True)
        np.testing.assert_allclose(fixed_transform(root, side + '_base'), baseline_meta['arms'][side]['world_from_base'], atol=1e-12)
        metadata['arms'][side] = dict(baseline_meta['arms'][side], control_urdf=path.name, control_sha256=digest(path))
    baseline_profile = baseline / 'collision.json'
    collision = json.loads(baseline_profile.read_text())
    collision['shapes'] = [s for s in collision['shapes'] if not is_tool(s['frame'])]
    kept = {s['frame'] for s in collision['shapes']}
    collision['allowed_link_pairs'] = [p for p in collision['allowed_link_pairs'] if set(p) <= kept]
    collision['kinematically_separated_pairs'] = [p for p in collision['kinematically_separated_pairs'] if set(p) <= kept]
    collision.pop('finger_fit', None)
    reasons = []
    certificates = prove_separation(tool / 'urdf/ag95.urdf', policy['separation_candidates'], collision['self_margin_m'])
    collision['separation_certificates'] = []
    for side in SIDES:
        for original in tool_profile['shapes']:
            shape = copy.deepcopy(original)
            shape.update(frame=f'{side}_tool_' + original['frame'], name=f'{side}_tool_' + original['name'], group=side)
            shape.pop('mesh', None)
            collision['shapes'].append(shape)
        for rule in policy['allowed_link_pairs']:
            pair = [f'{side}_tool_{name}' for name in rule['links']]
            collision['allowed_link_pairs'].append(pair)
            reasons.append(dict(rule, links=pair))
        for certificate in certificates:
            pair = [f'{side}_tool_{name}' for name in certificate['links']]
            collision['allowed_link_pairs'].append(pair)
            collision['kinematically_separated_pairs'].append(pair)
            collision['separation_certificates'].append(dict(certificate, links=pair))
    parent = {j.find('child').get('link'): j.find('parent').get('link') for j in root.findall('joint')}
    fixed = {j.find('child').get('link') for j in root.findall('joint') if j.get('type') == 'fixed'}
    def rigid(name):
        while name in fixed:
            name = parent[name]
        return name
    names = sorted({s['frame'] for s in collision['shapes']})
    for a, b in itertools.combinations(names, 2):
        if is_tool(a) == is_tool(b) or a.split('_')[0] != b.split('_')[0]:
            continue
        ra, rb = rigid(a), rigid(b)
        if ra == rb or (ra in parent and rigid(parent[ra]) == rb) or (rb in parent and rigid(parent[rb]) == ra):
            pair = [a, b]
            collision['allowed_link_pairs'].append(pair)
            reasons.append({'links': pair, 'reason': 'Rigid flange attachment or adjacent wrist/tool joint'})
    collision['allowed_link_pairs'] = sorted({tuple(sorted(p)) for p in collision['allowed_link_pairs']})
    collision['allowed_pair_reasons'] = reasons
    collision['allowed_pair_policy'] = 'Retained bench 08 arm/environment policy; explicit DH linkage connections; geometry-locked temporary fingertip exclusion; three source-separated pairs per tool proven over the full coupled range. Cross-arm and tool/environment pairs remain enabled.'
    collision['urdf_sha256'] = digest(derived)
    collision['sources'] = {str(Path(MODEL_ID) / name): value for name, value in files.items()}
    settings = json.loads((baseline / 'simulation_profile.json').read_text())
    settings['native_assembly'] = MODEL_ID
    settings['native_collision_gripper_calibration'] = {
        side: {f'{side}_tool_gripper_finger1_joint': [[0.0, 0.6524], [1.0, 0.0]]} for side in SIDES
    }
    for side in SIDES:
        settings[f'{side}_tool_transform'] = np.eye(4).ravel().tolist()
    settings['source'] = {'path': 'provenance/bench08_simulation_profile.json', 'sha256': digest(baseline / 'simulation_profile.json'),
                          'scope': 'Bench 08 arm settings; flange TCP and normalized URDF opening for offline DH testing, not measured hardware calibration'}
    settings['simulation_only'] = True
    manifest = {'source': 'xense-description', 'revision': subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip(),
                'source_assembly_id': record['id'], 'target_model': MODEL_ID, 'files_sha256': files,
                'baseline_collision_sha256': digest(baseline_profile), 'generator': 'scripts/import_dh_ag95.py'}
    for name, value in (('control.json', metadata), ('collision.json', collision), ('simulation_profile.json', settings),
                        ('simulation_recipe.json', {'robot': settings}), ('manifest.json', manifest)):
        (destination / name).write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')
    return len(collision['shapes'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--destination', type=Path, default=STATIONS / MODEL_ID)
    parser.add_argument('--check', action='store_true', help='Regenerate in a temporary directory and compare all files')
    args = parser.parse_args()
    if args.check:
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / MODEL_ID
            import_station(args.source, target)
            def hashes(root):
                return {str(p.relative_to(root)): digest(p) for p in root.rglob('*') if p.is_file()}
            if hashes(target) != hashes(args.destination):
                raise ValueError('Packaged DH snapshot differs from regeneration')
            print('DH snapshot reproduced byte-for-byte')
    else:
        print(f'Imported {import_station(args.source, args.destination)} collision shapes into {args.destination}')


if __name__ == '__main__':
    main()
