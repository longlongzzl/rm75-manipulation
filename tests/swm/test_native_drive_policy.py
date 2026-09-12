"""Native-shaped fixtures; actual PhysX readback is separately qualified."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.native_robot_mirror import (
    ARM_JOINTS, GRIPPER_JOINTS, articulation_drive_policy, align_articulation_drive_policy)
from rm75_app.swm.native_body_mirror import compare_native_state
from rm75_app.swm.scene import SceneInvalid


class Joint(SimpleNamespace):
    def set_drive_properties(self, stiffness, damping, force_limit, mode):
        self.stiffness, self.damping, self.force_limit, self.drive_mode = stiffness, damping, force_limit, mode
    def set_friction(self, value): self.friction = value
    def set_armature(self, value): self.armature = value.copy()


class Robot:
    def __init__(self):
        self.solver_position_iterations = 15
        self.solver_velocity_iterations = 1
        self.sleep_threshold = .005
        self.joints = [Joint(name=name, dof=1, type='revolute', limits=np.array([[0., .91]]),
            armature=np.array([0.]), drive_mode='force', stiffness=1000., damping=100.,
            force_limit=5., friction=1.) for name in ARM_JOINTS + GRIPPER_JOINTS]
    def get_active_joints(self): return self.joints
    def set_solver_position_iterations(self, value): self.solver_position_iterations = value
    def set_solver_velocity_iterations(self, value): self.solver_velocity_iterations = value
    def set_sleep_threshold(self, value): self.sleep_threshold = value


def test_original_active_and_passive_drives_are_copied_without_source_changes():
    source, private = Robot(), Robot()
    source.joints[-1].stiffness = 0.
    source.joints[-1].damping = 0.
    source.joints[-1].friction = 0.
    source.joints[0].armature = np.array([.125])
    source.joints[0].drive_mode = 'acceleration'
    source.solver_velocity_iterations = 16
    before = copy.deepcopy(articulation_drive_policy(source))
    expected = align_articulation_drive_policy(source, private)
    assert articulation_drive_policy(source) == before == expected
    assert articulation_drive_policy(private) == expected


@pytest.mark.parametrize('field,value', [('damping', float('nan')), ('friction', -1.),
    ('armature', np.array([-1.])), ('limits', np.array([[1.,0.]])), ('drive_mode','unknown')])
def test_invalid_native_policy_rejects_before_any_destination_write(field, value):
    source, private = Robot(), Robot()
    before = articulation_drive_policy(private)
    setattr(source.joints[0], field, value)
    with pytest.raises(SceneInvalid): align_articulation_drive_policy(source, private)
    assert articulation_drive_policy(private) == before


def test_primary_alias_is_rejected():
    source = Robot()
    with pytest.raises(SceneInvalid, match='independent'): align_articulation_drive_policy(source, source)
    private = Robot()
    private.joints[-1] = source.joints[-1]
    with pytest.raises(SceneInvalid, match='aliases'): align_articulation_drive_policy(source, private)


def test_joint_identity_mismatch_does_not_rewrite_geometry():
    source, private = Robot(), Robot()
    private.joints[0].limits = np.array([[-1.,1.]])
    before = articulation_drive_policy(private)
    with pytest.raises(SceneInvalid): align_articulation_drive_policy(source, private)
    assert articulation_drive_policy(private) == before


@pytest.mark.parametrize('field,value', [('damping', 2.), ('friction', .2), ('armature', np.array([.1]))])
def test_post_copy_native_drift_is_not_hidden_by_expected_cache(field, value):
    source, private = Robot(), Robot()
    expected = align_articulation_drive_policy(source, private)
    setattr(private.joints[3], field, value)
    with pytest.raises(SceneInvalid): compare_native_state(expected, articulation_drive_policy(private))


def test_silent_native_setter_failure_is_rejected():
    source, private = Robot(), Robot()
    source.joints[0].damping = 0.
    private.joints[0].set_drive_properties = lambda *args: None
    with pytest.raises(SceneInvalid): align_articulation_drive_policy(source, private)


def test_namespaced_native_source_uses_owned_wrapper_handles_not_suffixes():
    source, private = Robot(), Robot()
    canonical = {joint.name: joint for joint in source.joints}
    for joint in source.joints:
        joint.name = 'scene-0-RM75_' + joint.name
    source.joints.reverse()
    with pytest.raises(SceneInvalid): align_articulation_drive_policy(source, private)
    expected = align_articulation_drive_policy(source, private, source_joints=canonical)
    assert expected == articulation_drive_policy(private)
    assert all(joint.name.startswith('scene-0-RM75_') for joint in source.joints)


@pytest.mark.parametrize('bad', ['foreign', 'duplicate', 'missing', 'extra', 'native_duplicate'])
def test_canonical_map_must_cover_exact_owned_native_inventory(bad):
    source, private = Robot(), Robot()
    mapping = {joint.name: joint for joint in source.joints}
    if bad == 'foreign': mapping[ARM_JOINTS[0]] = Robot().joints[0]
    if bad == 'duplicate': mapping[ARM_JOINTS[0]] = source.joints[1]
    if bad == 'missing': mapping.pop(ARM_JOINTS[0])
    if bad == 'extra': mapping['extra'] = source.joints[0]
    if bad == 'native_duplicate': source.joints[0] = source.joints[1]
    before = articulation_drive_policy(private)
    with pytest.raises(SceneInvalid):
        align_articulation_drive_policy(source, private, source_joints=mapping)
    assert articulation_drive_policy(private) == before
