"""Bind original fitted payload geometry to measured transforms and GPU storage."""
from pathlib import Path
import hashlib

import numpy as np

from .scene import SceneInvalid, transform


def compare_attachment_spheres(expected, actual):
    expected, actual = np.asarray(expected, dtype=float), np.asarray(actual, dtype=float)
    if (expected.ndim != 2 or expected.shape[1:] != (4,) or not len(expected)
            or actual.ndim != 2 or actual.shape[1:] != (4,)
            or not np.isfinite(expected).all() or not np.isfinite(actual).all()
            or np.any(expected[:, 3] <= 0)):
        raise SceneInvalid('Finite positive original attachment spheres required')
    count = len(expected)
    if len(actual) < count or np.any(actual[count:, 3] >= 0):
        raise SceneInvalid('Native attachment slot coverage differs from original fit')
    if (not np.allclose(actual[:count, :3], expected[:, :3], atol=1e-5, rtol=0)
            or not np.allclose(actual[:count, 3], expected[:, 3], atol=1e-6, rtol=0)):
        raise SceneInvalid('Native attachment geometry or measured transform differs')
    return float(np.max(np.abs(actual[:count] - expected)))


def read_curobo_attachment_ack(backend, snapshot):
    from rm75_app.planning.contracts import JointConfiguration

    storage = backend.read_attachment_collision_state()
    if not storage['consumers_consistent'] or not storage['owners']:
        raise SceneInvalid('Native attachment planning consumers disagree')
    holding = snapshot['robot']['holding']
    if holding == 'empty':
        if any(any(count != 0 for count in row['active_counts']) for row in storage['owners']):
            raise SceneInvalid('Native GPU retains an unobserved payload')
        return dict(source=storage['source'], holding='empty', active_spheres=0,
                    attachment_geometry_qualified=True, actual_grasp_qualified=False)
    asset = snapshot['assets'][snapshot['objects'][holding]['asset_id']]
    reference = getattr(backend, '_swm_attachment_reference', None)
    if not reference:
        raise SceneInvalid('Native attachment fit reference is missing')
    path = Path(asset['mesh_path']).resolve(strict=True)
    if (reference['object_name'] != holding or reference['mesh_path'] != str(path)
            or reference['mesh_sha256'] != asset['mesh_sha256']
            or hashlib.sha256(path.read_bytes()).hexdigest() != asset['mesh_sha256']
            or reference['scale'] != [1., 1., 1.]):
        raise SceneInvalid('Native attachment source is not the registered metric asset')
    source = np.asarray(reference['object_spheres'], dtype=float)
    if source.ndim != 2 or source.shape[1:] != (4,):
        raise SceneInvalid('Original fitted sphere geometry is unavailable')
    world_object = transform(snapshot['objects'][holding]['measured']['T_world_object'])
    relative = np.linalg.inv(transform(snapshot['robot']['T_world_tcp'])) @ world_object

    def placed(matrix):
        result = source.copy()
        result[:, :3] = source[:, :3] @ matrix[:3, :3].T + matrix[:3, 3]
        return result

    local = placed(relative)
    errors = {}
    for row in storage['owners']:
        values = np.asarray(row['spheres'], dtype=float)
        if values.ndim != 3 or not len(values):
            raise SceneInvalid('Native attachment consumer storage is incomplete')
        errors[row['owner']] = [compare_attachment_spheres(local, value) for value in values]
    robot = snapshot['robot']
    current = JointConfiguration(tuple(robot['joint_names']), robot['positions'])
    world_spheres = backend.read_attachment_world_spheres(current)
    if np.asarray(world_spheres).shape[0] != 1:
        raise SceneInvalid('Native attachment world FK must have one measured state')
    world_error = compare_attachment_spheres(placed(world_object), world_spheres[0])
    return dict(source='native_GPU_payload_and_FK_readback', holding=holding,
        mesh_sha256=asset['mesh_sha256'], active_spheres=len(source),
        consumer_max_errors_m=errors, world_max_error_m=world_error,
        original_fit_binding_verified=True, attachment_geometry_qualified=True,
        actual_grasp_qualified=False)
