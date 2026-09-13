"""Native-contract fixtures, not physical qualification."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.native_velocity import NativeArticulationVelocity
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.native_robot_mirror import ARM_JOINTS, GRIPPER_JOINTS


def rig(sleeping=True):
    native = SimpleNamespace()
    names = list(ARM_JOINTS + GRIPPER_JOINTS)
    q = np.arange(13, dtype=float) / 10
    v = np.full(13, .002)
    links = [SimpleNamespace(name=f'link_{i}', is_root=i == 0, articulation=native,
        sleeping=sleeping, linear_velocity=np.zeros(3), angular_velocity=np.zeros(3)) for i in range(20)]
    native.root = links[0]
    native.get_links = lambda: links
    joints = [SimpleNamespace(name='scene-0_' + name) for name in names]
    native.get_active_joints = lambda: joints
    native.get_qpos = lambda: q.copy()
    native.get_qvel = lambda: v.copy()
    robot = SimpleNamespace(_objs=[native], joints_map={name: SimpleNamespace(_objs=[joint])
        for name, joint in zip(names, joints)})
    native.wrapper = robot
    observer = NativeArticulationVelocity(robot)
    return observer, native, names, q, v, links


def test_verified_sleep_retains_raw_cache_without_native_writes():
    observer, native, names, q, v, _ = rig()
    effective, evidence = observer(names, q, v)
    np.testing.assert_array_equal(effective, np.zeros(13))
    assert evidence['raw_native_velocities'] == v.tolist()
    assert evidence['source'] == 'native_PhysX_all_link_sleep_constraint'
    assert not evidence['native_state_mutated']
    np.testing.assert_array_equal(native.get_qvel(), v)


def test_awake_keeps_original_joint_velocity():
    observer, _, names, q, v, _ = rig(False)
    effective, evidence = observer(names, q, v)
    np.testing.assert_array_equal(effective, v)
    assert evidence['source'] == 'native_joint_velocity_readback'


def test_one_awake_link_cannot_prove_zero_velocity():
    observer, _, names, q, v, links = rig()
    links[-1].sleeping = False
    np.testing.assert_array_equal(observer(names, q, v)[0], v)


def test_sleeping_nonzero_spatial_velocity_is_rejected():
    observer, _, names, q, v, links = rig()
    links[-1].angular_velocity[0] = 1e-20
    with pytest.raises(SceneInvalid, match='nonzero spatial'):
        observer(names, q, v)


def test_raw_nonfinite_cache_is_not_hidden_by_sleep():
    observer, _, names, q, v, _ = rig()
    v[0] = np.nan
    with pytest.raises(SceneInvalid, match='finite'):
        observer(names, q, v)


def test_mismatched_joint_state_cannot_be_certified():
    observer, _, names, q, v, _ = rig()
    with pytest.raises(SceneInvalid, match='another state'):
        observer(names, q + .01, v)


def test_changed_link_inventory_is_rejected():
    observer, _, names, q, v, links = rig()
    links.pop()
    with pytest.raises(SceneInvalid, match='identity changed'):
        observer(names, q, v)


def test_reordered_joint_input_preserves_mapping_and_raw_evidence():
    observer, _, names, q, v, _ = rig()
    result, evidence = observer(names[::-1], q[::-1], v[::-1])
    assert evidence['joint_names'] == names[::-1]
    np.testing.assert_array_equal(result, np.zeros(13))


def test_native_joint_order_is_resolved_by_handle_not_name_or_position():
    _, native, names, q, v, _ = rig()
    joints = native.get_active_joints()[::-1]
    native.get_active_joints = lambda: joints
    native.get_qpos = lambda: q[::-1].copy()
    native.get_qvel = lambda: v[::-1].copy()
    observer = NativeArticulationVelocity(native.wrapper)
    effective, evidence = observer(names, q, v)
    np.testing.assert_array_equal(effective, np.zeros(13))
    assert evidence['raw_native_velocities'] == v.tolist()


@pytest.mark.parametrize('defect', ['missing', 'foreign', 'duplicate'])
def test_invalid_canonical_handle_mapping_rejected(defect):
    _, native, names, _, _, _ = rig()
    if defect == 'missing':
        del native.wrapper.joints_map[names[0]]
    elif defect == 'foreign':
        native.wrapper.joints_map[names[0]]._objs = [SimpleNamespace(name='scene-0_' + names[0])]
    else:
        native.wrapper.joints_map[names[0]]._objs = native.wrapper.joints_map[names[1]]._objs
    with pytest.raises(SceneInvalid):
        NativeArticulationVelocity(native.wrapper)
