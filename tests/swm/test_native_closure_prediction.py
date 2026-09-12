"""The prediction is a veto, not a substitute for existing execution guards."""
from types import SimpleNamespace

import pytest

from rm75_app.swm.native_closure import forbidden_object_contact
from rm75_app.swm.native_execution import NativePrimaryExecutor
from rm75_app.execution.trajectory_executor import ManiSkillTrajectoryExecutor
from rm75_app.swm.scene import SceneInvalid


def classify(names, force=(0., 0., 0.), separation=(.01,)):
    return forbidden_object_contact(names, force, separation,
        objects={'bi', 'table'}, fingers={'left', 'right'}, target='bi')


@pytest.mark.parametrize('force,separation,expected', [
    ((0., 0., 0.), (.01,), False),
    ((0., 0., 1e-15), (.01,), True),
    ((0., 0., 0.), (-1e-15,), True),
])
def test_forbidden_table_contact_retains_small_force_and_zero_impulse_penetration(force, separation, expected):
    assert classify(['left', 'table'], force, separation) is expected


def test_only_original_target_fingers_are_allowed():
    assert not classify(['bi', 'right'], (1., 0., 0.), (-.001,))
    assert classify(['arm', 'bi'], (1., 0., 0.), (-.001,))
    assert not classify(['arm', 'left'], (1., 0., 0.), (-.001,))


@pytest.mark.parametrize('force,separation', [((float('nan'), 0., 0.), (.01,)),
    ((0., 0.), (.01,)), ((0., 0., 0.), (None,)), ((0., 0., 0.), ())])
def test_incomplete_native_contact_refuses(force, separation):
    with pytest.raises(SceneInvalid):
        classify(['left', 'table'], force, separation)


def test_prediction_veto_prevents_shared_close_and_settle(monkeypatch):
    calls = []
    sink = object.__new__(NativePrimaryExecutor)
    sink.primary = SimpleNamespace(stop=SimpleNamespace(check=lambda: None))
    sink._last_commanded_target = [0.] * 7
    sink._after_control_step = lambda stage: calls.append('original_guard')
    sink._settle = lambda stage: calls.append('settle')
    def reject():
        calls.append('prediction')
        raise SceneInvalid('predicted forbidden contact')
    sink.closure_prediction = reject
    monkeypatch.setattr(ManiSkillTrajectoryExecutor, 'set_gripper', lambda *args: calls.append('command'))
    with pytest.raises(SceneInvalid):
        sink.set_gripper(True)
    assert calls == ['original_guard', 'prediction']


def test_open_does_not_run_close_prediction(monkeypatch):
    calls = []
    sink = object.__new__(NativePrimaryExecutor)
    sink.primary = SimpleNamespace(stop=SimpleNamespace(check=lambda: None))
    sink._last_commanded_target = [0.] * 7
    sink.closure_prediction = lambda: calls.append('prediction')
    sink._settle = lambda stage: calls.append(stage)
    monkeypatch.setattr(ManiSkillTrajectoryExecutor, 'set_gripper', lambda *args: calls.append('command'))
    sink.set_gripper(False)
    assert calls == ['command', 'gripper_open']
