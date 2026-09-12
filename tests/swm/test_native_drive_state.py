"""Effective drive targets are neither measured q nor controller replay."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.native_robot import read_primary_drive_state
from rm75_app.swm.native_robot_mirror import (ARM_JOINTS, GRIPPER_JOINTS,
    validate_native_drive_state, apply_native_drive_state)
from rm75_app.swm.scene import SceneInvalid


class Joint(SimpleNamespace):
    def set_drive_target(self, value): self.drive_target = value.copy()
    def set_drive_velocity_target(self, value): self.drive_velocity_target = value.copy()


def rig():
    names = ARM_JOINTS + GRIPPER_JOINTS
    raw = [Joint(name='scene-0-RM75_'+name, drive_target=np.array([.25]),
        drive_velocity_target=np.array([.01])) for name in names]
    native = SimpleNamespace(get_active_joints=lambda: raw)
    wrappers = {name: SimpleNamespace(_objs=[joint]) for name,joint in zip(names,raw)}
    env = SimpleNamespace(agent=SimpleNamespace(robot=SimpleNamespace(_objs=[native],joints_map=wrappers)),
        scene=SimpleNamespace(px=SimpleNamespace(timestep=.01)), sim_freq=100, control_freq=20)
    primary = SimpleNamespace(closed=False, stop=SimpleNamespace(check=lambda:None), env=SimpleNamespace(unwrapped=env))
    private = [Joint(name=name, drive_target=np.zeros(1),drive_velocity_target=np.zeros(1)) for name in names]
    return primary, raw, SimpleNamespace(get_active_joints=lambda: private), SimpleNamespace(timestep=.02)


def test_native_targets_are_read_and_applied_separately_from_measured_positions():
    primary, raw, private, system = rig()
    state = read_primary_drive_state(primary)
    before = copy.deepcopy(state)
    result = apply_native_drive_state(private, system, state)
    assert state == before == read_primary_drive_state(primary)
    assert result['position_targets'] == [.25]*13
    np.testing.assert_allclose(result['velocity_targets'], [.01]*13)
    assert system.timestep == .01
    assert result['drive_targets_aligned'] and not result['controller_state_replay_qualified']
    raw[0].drive_target[0] = .5
    assert read_primary_drive_state(primary)['position_targets'][0] == .5


@pytest.mark.parametrize('bad', ['missing','source','nan','timing','ratio','names','velocity'])
def test_bad_drive_state_is_rejected_before_private_writes(bad):
    primary, _, private, system = rig()
    state = read_primary_drive_state(primary)
    if bad=='missing': state.pop('position_targets')
    if bad=='source': state['source']='command_cache'
    if bad=='nan': state['position_targets'][0]=float('nan')
    if bad=='timing': state['physics_timestep_s']=.02
    if bad=='ratio': state['control_frequency_hz']=30
    if bad=='names': state['joint_names'][0]=state['joint_names'][1]
    if bad=='velocity': state['velocity_targets'].pop()
    with pytest.raises(SceneInvalid): apply_native_drive_state(private, system, state)
    assert system.timestep==.02
    assert all(joint.drive_target[0]==0 for joint in private.get_active_joints())


def test_silent_native_target_setter_failure_is_detected():
    primary, _, private, system = rig()
    private.get_active_joints()[0].set_drive_target=lambda value:None
    with pytest.raises(SceneInvalid):
        apply_native_drive_state(private, system, read_primary_drive_state(primary))
