"""Pure contract negatives; actual CPU articulation evidence is separate."""
import copy

import numpy as np
import pytest

from rm75_app.swm.native_robot_mirror import ARM_JOINTS, GRIPPER_JOINTS, measured_joint_vectors
from rm75_app.swm.scene import SceneInvalid


def observation():
    return dict(idle=True, units='rad', joint_names=list(ARM_JOINTS),
        positions=list(np.arange(7) / 10), velocities=[0.] * 7,
        gripper_positions={name: i / 100 for i, name in enumerate(GRIPPER_JOINTS)},
        gripper_velocities={name: i / 100000 for i, name in enumerate(GRIPPER_JOINTS)})


def test_measured_joint_order_preserves_independent_jaw_and_velocity():
    state = observation()
    original = copy.deepcopy(state)
    names = tuple(reversed(ARM_JOINTS + GRIPPER_JOINTS))
    q, v = measured_joint_vectors(state, names)
    assert state == original
    for name in GRIPPER_JOINTS:
        assert q[names.index(name)] == state['gripper_positions'][name]
        assert v[names.index(name)] == state['gripper_velocities'][name]


@pytest.mark.parametrize('bad', ['missing_jaw', 'missing_velocity', 'moving', 'nan', 'duplicate', 'units'])
def test_incomplete_or_moving_measured_robot_is_rejected(bad):
    state = observation()
    names = ARM_JOINTS + GRIPPER_JOINTS
    if bad == 'missing_jaw':
        state['gripper_positions'].pop(GRIPPER_JOINTS[0])
    elif bad == 'missing_velocity':
        state.pop('velocities')
    elif bad == 'moving':
        state['gripper_velocities'][GRIPPER_JOINTS[0]] = .002
    elif bad == 'nan':
        state['positions'][0] = float('nan')
    elif bad == 'duplicate':
        names = names[:-1] + (names[0],)
    else:
        state['units'] = 'deg'
    with pytest.raises(SceneInvalid):
        measured_joint_vectors(state, names)
