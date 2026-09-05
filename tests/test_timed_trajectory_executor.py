import numpy as np
import pytest

from rm75_app.execution.trajectory_executor import (
    ManiSkillTrajectoryExecutor, sample_timed_joint_path,
)
from rm75_app.planning.contracts import JointTrajectory


def test_timed_sampling_preserves_duration_without_initial_hold():
    path = JointTrajectory(("j",), np.linspace(0, 1, 41)[:, None], dt=0.025)
    sampled = sample_timed_joint_path(path, 0.05)
    assert len(sampled) == 20
    np.testing.assert_allclose(sampled[:, 0], np.arange(1, 21) / 20)


def test_variable_intervals_and_fractional_last_tick():
    path = JointTrajectory(("j",), np.array([[0], [1], [2]]), dt=np.array([0.1, 0.15]))
    np.testing.assert_allclose(sample_timed_joint_path(path, 0.1)[:, 0], [1, 5/3, 2])


@pytest.mark.parametrize("dt", [None, 0, -1, np.nan, np.inf, [0.1, 0.2]])
def test_invalid_timing_rejected_before_any_action(dt):
    class Sink:
        def compose_action(self, *args):
            pytest.fail("invalid timing reached executor")
    executor = ManiSkillTrajectoryExecutor(Sink(), control_dt=0.05)
    with pytest.raises(ValueError):
        executor.execute_trajectory("move", JointTrajectory(("j",), np.array([[0], [1]]), dt=dt))


def test_timed_execution_ignores_legacy_downsampling_cap():
    actions = []
    class Sink:
        def compose_action(self, positions, gripper):
            return positions
        def step_and_render(self, action, tag):
            actions.append(action)
    executor = ManiSkillTrajectoryExecutor(Sink(), control_dt=0.05, max_path_points=2)
    executor.execute_trajectory("move", JointTrajectory(("j",), np.array([[0], [1]]), dt=1.0))
    assert len(actions) == 20
    np.testing.assert_allclose(actions[-1], [1])
