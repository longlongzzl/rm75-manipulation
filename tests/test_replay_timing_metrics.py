import pytest

from rm75_app.validation.maniskill_gate import summarize_replay_timing


def event(kind, start, end, stage="move"):
    return {"kind": kind, "start_s": start, "end_s": end, "stage": stage}


def test_excludes_only_declared_dwell_and_checkpoint():
    result = summarize_replay_timing([
        event("trajectory", 0, 1, "a"), event("gripper", 1.02, 2.02),
        event("checkpoint", 2.03, 2.53), event("trajectory", 2.6, 3, "b")])
    gap = result["adjacent_motion_gaps"][0]
    assert gap["wall_gap_s"] == pytest.approx(1.6)
    assert gap["excluded_dwell_checkpoint_s"] == 1.5
    assert gap["software_idle_s"] == pytest.approx(0.1)
    assert result["p95_below_150ms"] is True


def test_empty_and_single_motion_are_not_timing_passes():
    assert summarize_replay_timing([])["p95_below_150ms"] is None
    assert summarize_replay_timing([event("trajectory", 0, 1)])["software_idle_p95_s"] is None


def test_unexplained_gap_remains_failure():
    result = summarize_replay_timing([event("trajectory", 0, 1), event("trajectory", 1.2, 2)])
    assert result["p95_below_150ms"] is False


def test_overlapping_timing_is_rejected():
    with pytest.raises(ValueError, match="overlapping"):
        summarize_replay_timing([event("trajectory", 0, 1), event("gripper", 1, 3), event("trajectory", 2, 4)])
