from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.native_drive_commands import apply_private_control, needs_group_write
from rm75_app.swm.native_robot_mirror import ARM_JOINTS, GRIPPER_JOINTS
from rm75_app.swm.scene import SceneInvalid


def rig():
    writes = []
    class Joint:
        def __init__(self, name):
            self.name = name
            self.drive_target = np.zeros(1, dtype=np.float32)
            self.drive_velocity_target = np.zeros(1, dtype=np.float32)
        def set_drive_target(self, value):
            writes.append(self.name)
            self.drive_target = value.copy()
    names = list(ARM_JOINTS + GRIPPER_JOINTS)
    joints = [Joint(name) for name in names]
    robot = SimpleNamespace(get_active_joints=lambda: joints)
    state = dict(source='native_joint_drive_targets_readback', joint_names=names,
        position_targets=[0.]*13, velocity_targets=[0.]*13,
        physics_timestep_s=float(np.float32(.01)), simulation_frequency_hz=100., control_frequency_hz=20.)
    groups = (('arm', ARM_JOINTS), ('gripper', (GRIPPER_JOINTS[0], GRIPPER_JOINTS[3])))
    return robot, SimpleNamespace(timestep=state['physics_timestep_s']), state, groups, writes


def test_private_writes_whole_changed_group_not_passive_or_velocity_targets():
    robot, physics, state, groups, writes = rig()
    state['position_targets'][0] = .1
    result = apply_private_control(robot, physics, state, groups)
    assert writes == list(ARM_JOINTS)
    assert result['gripper']['exact_equal_skipped']
    writes.clear()
    apply_private_control(robot, physics, state, groups)
    assert writes == []


def test_native_precision_difference_is_not_tolerance_skipped():
    robot, _, _, _, _ = rig()
    joint = robot.get_active_joints()[0]
    assert needs_group_write([joint], [np.nextafter(np.float32(0), np.float32(1))])


@pytest.mark.parametrize('defect', ['passive', 'velocity', 'timestep'])
def test_private_control_never_repairs_changed_initialization_state(defect):
    robot, physics, state, groups, writes = rig()
    if defect == 'passive':
        state['position_targets'][-1] = .1
    elif defect == 'velocity':
        state['velocity_targets'][0] = .1
    else:
        physics.timestep = .02
    with pytest.raises(SceneInvalid):
        apply_private_control(robot, physics, state, groups)
    assert writes == []
