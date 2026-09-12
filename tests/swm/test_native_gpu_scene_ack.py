"""GPU-storage protocol fixtures, not native runtime qualification."""
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.swm.native_planning_scene import read_curobo_collision_ack
from rm75_app.swm.scene import SceneInvalid


def model():
    data = SimpleNamespace(count=np.array([1]), names=[['box']],
        dims=np.array([[[1., 2., 3., 0.]]]),
        inv_pose=np.array([[[-.2, -.3, -.4, 1., 0., 0., 0., 0.]]]),
        enable=np.array([[1]]))
    checker = SimpleNamespace(data=SimpleNamespace(cuboids=data),
        get_obstacle_names=lambda: ['box'])
    backend = SimpleNamespace(_ensure_planner=lambda: SimpleNamespace(scene_collision_checker=checker),
        _to_formal_curobo_scene_dict=lambda scene: {'cuboid': {
            'box': {'pose': [.2, .3, .4, 1., 0., 0., 0.], 'dims': [1., 2., 3.]}}})
    return backend, data, checker


def test_ack_uses_native_tensor_pose_dimensions_and_enable():
    backend, _, _ = model()
    result = read_curobo_collision_ack(backend, SimpleNamespace(revision='measured-scene'))
    assert result['scene_revision'] == 'measured-scene'
    assert result['count'] == 1
    np.testing.assert_allclose(np.asarray(result['obstacles'][0]['T_base_proxy'])[:3, 3], [.2, .3, .4])
    assert result['robot_state_qualified'] is False
    assert result['attachment_qualified'] is False


@pytest.mark.parametrize('fault', ['disabled', 'pose', 'dimensions', 'identity', 'count'])
def test_cached_scene_name_cannot_hide_native_tensor_fault(fault):
    backend, data, checker = model()
    if fault == 'disabled':
        data.enable[0, 0] = 0
    elif fault == 'pose':
        data.inv_pose[0, 0, 0] += .1
    elif fault == 'dimensions':
        data.dims[0, 0, 0] *= .9
    elif fault == 'identity':
        checker.get_obstacle_names = lambda: ['another-object']
    else:
        data.count[0] = 0
    with pytest.raises(SceneInvalid):
        read_curobo_collision_ack(backend, SimpleNamespace(revision='measured-scene'))
