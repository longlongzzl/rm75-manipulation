import pytest

from rm75_app.scenarios.pickplace_program import PickPlaceProgramExecutor
from test_review_world_model_safety import Sink, compiled_program


def fail(*args):
    raise RuntimeError("observer unavailable")


def test_scene_feedback_exception_blocks_all_dispatch():
    sink = Sink()
    result = PickPlaceProgramExecutor(sink, scene_stamp_provider=fail).execute(compiled_program())
    assert not result.success and not sink.commands
    assert result.failed_stage == "initial_state_validation"
    assert "observer unavailable" in result.message


@pytest.mark.parametrize("field,stage", [("pre_release_gate", "release_readiness"),
                                         ("atom_validator", "atom_outcome_validation")])
def test_callback_exception_stops_without_claiming_task_success(field, stage):
    sink, stopped = Sink(), []
    result = PickPlaceProgramExecutor(sink, **{field: fail},
                                     stop_callback=lambda: stopped.append(True)).execute(compiled_program())
    assert not result.success and not result.completed_atoms
    assert result.failed_stage == stage and stopped == [True]
    assert not result.diagnostics["task_success_verified"]
    if field == "pre_release_gate":
        assert ("gripper", False) not in sink.commands


@pytest.mark.parametrize("immediate", [True, False])
def test_cancel_stops_at_next_command_boundary(immediate):
    sink, stopped = Sink(), []
    result = PickPlaceProgramExecutor(
        sink, cancellation_requested=lambda: immediate or bool(sink.commands),
        stop_callback=lambda: stopped.append(True)).execute(compiled_program())
    assert result.failed_stage == "cancelled" and not result.success
    assert stopped == [True]
    assert len(sink.commands) == (0 if immediate else 1)
    assert not any(c[0] == "gripper" for c in sink.commands)


def test_stop_callback_error_does_not_replace_original_cancel():
    result = PickPlaceProgramExecutor(Sink(), cancellation_requested=lambda: True,
                                     stop_callback=fail).execute(compiled_program())
    assert result.failed_stage == "cancelled" and not result.success
    assert "observer unavailable" in result.diagnostics["stop_callback_error"]
