import numpy as np
import pytest
from rm75_app.swm.native_feedback_closure import NativeFeedbackClosure
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.native_execution import NativePrimaryExecutor


def test_ramp_hold_and_overforce_are_bounded_not_holding_proof():
    policy = NativeFeedbackClosure()
    assert policy.advance(np.zeros((2, 3))) == pytest.approx(-.96)
    before = policy.command
    assert policy.advance([[1., 0, 0], [0, 1., 0]]) == before
    assert policy.last['in_force_band'] is True
    assert policy.last['holding_qualified'] is False
    assert policy.advance([[5., 0, 0], [0, 5., 0]]) < before


@pytest.mark.parametrize('forces', [[], [1., 1.], [[float('nan'), 0, 0]] * 2,
                                    [[float('inf'), 0, 0]] * 2])
def test_invalid_native_feedback_rejected_without_advancing(forces):
    policy = NativeFeedbackClosure()
    with pytest.raises(SceneInvalid):
        policy.advance(forces)
    assert policy.ticks == 0


def test_original_total_budget_and_action_trace():
    policy = NativeFeedbackClosure()
    for _ in range(220):
        policy.advance(np.zeros((2, 3)))
    assert policy.ticks == 220
    assert policy.command == 1.
    assert policy.last['in_force_band'] is False
    with pytest.raises(SceneInvalid, match='budget'):
        policy.advance(np.zeros((2, 3)))


@pytest.mark.parametrize('paired',[False,True])
def test_postclosure_trajectory_cannot_reuse_old_audit(paired):
    from types import SimpleNamespace
    sink = object.__new__(NativePrimaryExecutor)
    sink.primary = SimpleNamespace(stop=SimpleNamespace(check=lambda: None))
    sink._closure_requires_reaudit = True
    sink.paired_observation_mode = paired
    with pytest.raises(SceneInvalid, match='re-audit'):
        sink.execute_trajectory('lift', None)
