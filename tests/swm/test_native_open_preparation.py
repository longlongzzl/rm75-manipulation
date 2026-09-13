"""Original command semantics and strict private opening feedback criteria."""
import copy
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.swm.native_closure import (
    NativeClosureRejected, original_gripper_drive_targets, settle_candidate_open)
from rm75_app.swm.native_execution import native_endpoint_metrics
from rm75_app.swm.scene import SceneInvalid


NAMES = [f'joint_{i}' for i in range(1, 8)] + [
    f'gripper_{side}_{part}_Joint' for side in ('Left', 'Right') for part in ('1', '2', 'Support')]


def test_original_open_and_close_transform_never_mutates_controller_or_baseline(monkeypatch):
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(
        ones=lambda shape, device: np.ones(shape), zeros_like=np.zeros_like))
    controller = SimpleNamespace(
        config=SimpleNamespace(use_delta=False, use_target=False, interpolate=False),
        effective_dof=1, device='fixture', _target_qpos=np.full((1, 2), .77),
        control_joint_indices=[0], mimic_joint_indices=[1], mimic_control_joint_indices=[0],
        _multiplier=np.array([1.]), _offset=np.array([0.]),
        joints=[SimpleNamespace(name='gripper_Left_1_Joint'), SimpleNamespace(name='gripper_Right_1_Joint')],
        _preprocess_action=lambda action: (action+1.)*.455)
    baseline = dict(source='native_joint_drive_targets_readback', joint_names=NAMES,
        position_targets=[.123]*13, velocity_targets=[0.]*13, physics_timestep_s=.01,
        simulation_frequency_hz=100., control_frequency_hz=20.)
    original = copy.deepcopy(baseline)
    for value, expected in [(-1., 0.), (1., .91)]:
        result = original_gripper_drive_targets(controller, baseline, value)
        for name in ('gripper_Left_1_Joint', 'gripper_Right_1_Joint'):
            assert result['position_targets'][NAMES.index(name)] == pytest.approx(expected)
        assert result['position_targets'][0] == .123
        assert result['position_targets'][NAMES.index('gripper_Left_2_Joint')] == .123
    assert baseline == original
    assert np.array_equal(controller._target_qpos, np.full((1, 2), .77))
    controller.config.interpolate = True
    with pytest.raises(SceneInvalid):
        original_gripper_drive_targets(controller, baseline, -1.)


def test_private_open_requires_three_consecutive_complete_idle_readbacks():
    calls = []
    def step(tick):
        calls.append(tick)
        velocity = np.zeros(13)
        if tick == 2:
            velocity[-1] = .002
        return dict(q=np.zeros(13), qdot=velocity)
    result = settle_candidate_open(step, NAMES, np.zeros(7), max_steps=6)
    assert calls == list(range(6))
    assert result['completed'] and result['stable_steps'] == 3
    assert result['source'] == 'native_private_open_hold_readback'


@pytest.mark.parametrize('joint', [0, 12])
def test_private_open_does_not_ignore_arm_or_passive_jaw_motion(joint):
    velocity = np.zeros(13)
    velocity[joint] = .002
    with pytest.raises(NativeClosureRejected) as caught:
        settle_candidate_open(lambda tick: dict(q=np.zeros(13), qdot=velocity),
                              NAMES, np.zeros(7), max_steps=3)
    assert caught.value.evidence['completed'] is False
    assert caught.value.evidence['max_velocity_rad_s'] == .002


def test_zero_velocity_does_not_override_arm_endpoint_error():
    q = np.zeros(13)
    q[0] = .021
    with pytest.raises(NativeClosureRejected):
        settle_candidate_open(lambda tick: dict(q=q, qdot=np.zeros(13)),
                              NAMES, np.zeros(7), max_steps=3)


def test_missing_feedback_is_model_error_not_candidate_infeasibility():
    with pytest.raises(SceneInvalid) as caught:
        settle_candidate_open(lambda tick: dict(q=np.zeros(7), qdot=np.zeros(13)),
                              NAMES, np.zeros(7), max_steps=3)
    assert type(caught.value) is SceneInvalid


def test_original_metrics_keep_exact_velocity_and_position_boundaries():
    velocity = np.zeros(13)
    velocity[-1] = .001
    error, speed, idle = native_endpoint_metrics(np.full(7, .02), velocity, np.zeros(7))
    assert error == .02 and speed == .001 and idle
    velocity[-1] += 1e-12
    assert not native_endpoint_metrics(np.zeros(7), velocity, np.zeros(7))[2]
