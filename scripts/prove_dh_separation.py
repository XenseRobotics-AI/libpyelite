"""Certify source-mesh separation over the complete coupled AG95 opening range."""
import xml.etree.ElementTree as ET

import coal
import numpy as np
import pinocchio as pin


def prove_separation(urdf, pairs, clearance):
    tree = ET.parse(urdf).getroot()
    joints = {j.get('name'): j for j in tree.findall('joint')}
    driver = 'gripper_finger1_joint'
    limit = joints[driver].find('limit')
    lower, upper = float(limit.get('lower')), float(limit.get('upper'))
    coefficients = {driver: (1., 0.)}
    def coefficient(name):
        if name not in coefficients:
            mimic = joints[name].find('mimic')
            gain, offset = coefficient(mimic.get('joint'))
            m = float(mimic.get('multiplier', 1))
            coefficients[name] = (gain * m, offset * m + float(mimic.get('offset', 0)))
        return coefficients[name]
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    geometry = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.VISUAL, package_dirs=[str(urdf.parent)])
    geometry_data = geometry.createData()
    request = coal.DistanceRequest()
    request.gjk_tolerance = 1e-10
    index = {model.frames[g.parentFrame].name: i for i, g in enumerate(geometry.geometryObjects)}
    if len(index) != len(geometry.geometryObjects):
        raise ValueError('DH separation proof requires one source visual mesh per link; review changed fingertips')
    speed = {}
    for name in {name for pair in pairs for name in pair}:
        g = geometry.geometryObjects[index[name]]
        reach = np.linalg.norm(g.geometry.vertices() @ g.placement.rotation.T + g.placement.translation, axis=1).max()
        joint, bound = g.parentJoint, 0.
        while joint:
            bound += abs(coefficient(model.names[joint])[0]) * reach
            reach += np.linalg.norm(model.jointPlacements[joint].translation)
            joint = model.parents[joint]
        speed[name] = float(bound)
    def distance(center, first, second):
        q = pin.neutral(model)
        for name in model.names[1:]:
            joint = model.joints[model.getJointId(name)]
            gain, offset = coefficient(name)
            angle = center * gain + offset
            q[joint.idx_q:joint.idx_q + joint.nq] = [np.cos(angle), np.sin(angle)] if joint.nq == 2 else [angle]
        pin.updateGeometryPlacements(model, data, geometry, geometry_data, q)
        i, j = index[first], index[second]
        a, b = geometry_data.oMg[i], geometry_data.oMg[j]
        return coal.distance(geometry.geometryObjects[i].geometry, coal.Transform3s(a.rotation, a.translation),
                             geometry.geometryObjects[j].geometry, coal.Transform3s(b.rotation, b.translation),
                             request, coal.DistanceResult())
    certificates = []
    for a, b in pairs:
        # A vertex moves at most radius * |angle change|. Summing lever-arm
        # bounds up each chain includes every mimic rotation. Distance between
        # the two complete meshes is therefore Lipschitz with speed[a]+speed[b].
        bound = speed[a] + speed[b]
        cells = [((lower + upper) / 2, (upper - lower) / 2, 0)]
        visits, minimum = 0, float('inf')
        while cells:
            center, half, depth = cells.pop()
            measured = distance(center, a, b)
            conservative = measured - bound * half - 1e-7
            visits += 1
            if not np.isfinite(measured) or measured <= clearance or visits > 100000 or depth > 24:
                raise ValueError(f'Cannot certify DH source separation: {a}, {b}')
            if conservative > clearance:
                minimum = min(minimum, conservative)
            else:
                cells.extend(((center - half / 2, half / 2, depth + 1),
                              (center + half / 2, half / 2, depth + 1)))
        certificates.append({'links': [a, b], 'clearance_m': clearance, 'lower_bound_m': minimum,
                             'driver_range_rad': [lower, upper], 'distance_lipschitz_m_per_rad': bound,
                             'intervals_checked': visits,
                             'reason': 'Source visual meshes separated over full coupled range by interval motion bounds'})
    return certificates
