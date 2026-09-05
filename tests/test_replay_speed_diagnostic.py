from pathlib import Path

import numpy as np
import pytest

from rm75_app.planning.contracts import JointTrajectory


@pytest.mark.parametrize("dt", [0.025, np.array([0.025, 0.05])])
def test_retiming_preserves_path_events_and_source(monkeypatch, dt):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_replay_speed import scale_events
    trajectory = JointTrajectory(("a",), np.array([[0.], [1.], [2.]]), dt)
    events = [{"type": "trajectory", "stage": "place", "trajectory": trajectory},
              {"type": "gripper", "closed": False}]
    result, audit = scale_events(events, 2)
    np.testing.assert_array_equal(result[0]["trajectory"].positions, trajectory.positions)
    np.testing.assert_array_equal(result[0]["trajectory"].dt, np.asarray(dt)*2)
    np.testing.assert_array_equal(trajectory.dt, dt)
    assert result[1] == events[1]
    assert audit[0]["positions_unchanged"]
    assert result[0]["trajectory"] is not trajectory


@pytest.mark.parametrize("factor", [0, 0.5, float("nan"), float("inf")])
def test_speedup_or_invalid_factor_rejected(monkeypatch, factor):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_replay_speed import scale_events
    with pytest.raises(ValueError, match="factor"):
        scale_events([], factor)


def test_missing_timing_not_invented(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_replay_speed import scale_events
    with pytest.raises(ValueError):
        scale_events([{"type": "trajectory", "stage": "place",
                       "trajectory": JointTrajectory(("a",), [[0.], [1.]])}], 2)
