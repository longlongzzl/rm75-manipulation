import numpy as np
import pytest
from rm75_app.swm.native_closure import settle_closed_hold, NativeClosureRejected
from rm75_app.swm.native_robot_mirror import ARM_JOINTS, GRIPPER_JOINTS
from rm75_app.swm.scene import SceneInvalid

NAMES = list(ARM_JOINTS + GRIPPER_JOINTS)

def row(moving=False):
    return dict(q=np.zeros(13), qdot=np.zeros(13), object_velocities={'bi': dict(
        linear_velocity=[0., 0., 0.], angular_velocity=[.006 if moving else 0., 0., 0.])})

def test_closed_hold_requires_full_object_and_robot_idle_three_times():
    calls = []
    def step(tick):
        calls.append(tick)
        return row(tick < 4)
    result = settle_closed_hold(step, NAMES, np.zeros(7), ['bi'], max_steps=7)
    assert calls == list(range(7))
    assert result['completed'] and not result['holding_qualified']

def test_late_contact_after_original_twenty_close_steps_is_not_missed():
    def step(tick):
        if tick == 78:
            raise NativeClosureRejected('late table contact')
        return row(True)
    with pytest.raises(NativeClosureRejected, match='late table contact'):
        settle_closed_hold(step, NAMES, np.zeros(7), ['bi'])

def test_original_closed_hold_budget_not_extended():
    calls = []
    def step(tick):
        calls.append(tick)
        return row(True)
    with pytest.raises(NativeClosureRejected) as caught:
        settle_closed_hold(step, NAMES, np.zeros(7), ['bi'])
    assert len(calls) == 200
    assert caught.value.evidence['completed'] is False

def test_missing_object_feedback_is_model_error_not_candidate_rejection():
    with pytest.raises(SceneInvalid) as caught:
        settle_closed_hold(lambda tick: row(), NAMES, np.zeros(7), ['bi', 'table'])
    assert type(caught.value) is SceneInvalid

def test_passive_joint_motion_still_prevents_closed_idle():
    def step(tick):
        value = row()
        value['qdot'][-1] = .002
        return value
    with pytest.raises(NativeClosureRejected):
        settle_closed_hold(step, NAMES, np.zeros(7), ['bi'], max_steps=3)
