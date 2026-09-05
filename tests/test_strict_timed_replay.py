import numpy as np
import pytest

from rm75_app.planning.contracts import JointConfiguration, JointTrajectory
from rm75_app.validation.maniskill_gate import validate_timed_replay_events


def events(*, second_start=0.1, second_dt=0.025, names=("j",)):
    return [{"type": "trajectory", "trajectory": JointTrajectory(
        names, np.array([[0.0], [0.1]]), dt=0.025)},
        {"type": "trajectory", "trajectory": JointTrajectory(
        names, np.array([[second_start], [0.2]]), dt=second_dt)}]


def test_accepts_actual_matching_initial_state():
    assert validate_timed_replay_events(events(), JointConfiguration(("j",), [0.01]), 0.05) == 0.01


def test_rejects_initial_gap_without_teleport():
    with pytest.raises(ValueError, match="no teleport"):
        validate_timed_replay_events(events(), JointConfiguration(("j",), [0.5]), 0.05)


def test_validates_later_trajectory_timing_before_replay():
    with pytest.raises(ValueError, match="explicit dt"):
        validate_timed_replay_events(events(second_dt=None), JointConfiguration(("j",), [0]), 0.05)


def test_rejects_inter_stage_gap():
    with pytest.raises(ValueError, match="stage gap"):
        validate_timed_replay_events(events(second_start=0.5), JointConfiguration(("j",), [0]), 0.05)


def test_rejects_wrong_joint_order():
    with pytest.raises(ValueError, match="joint names"):
        validate_timed_replay_events(events(names=("other",)), JointConfiguration(("j",), [0]), 0.05)


def test_rejects_empty_program():
    with pytest.raises(ValueError, match="no trajectories"):
        validate_timed_replay_events([], JointConfiguration(("j",), [0]), 0.05)
