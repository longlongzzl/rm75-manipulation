from __future__ import annotations

import json

import numpy as np
import pytest

from rm75_app.runtime.curobo2_sim_replay import load_replay_events


def test_load_replay_events_is_planner_independent(tmp_path) -> None:
    np.savez_compressed(
        tmp_path / "00_approach.npz",
        positions=np.asarray([[0.0, 1.0], [2.0, 3.0]]),
        joint_names=np.asarray(["j1", "j2"]),
        dt=np.asarray(0.05),
    )
    manifest = tmp_path / "execution.json"
    manifest.write_text(
        json.dumps(
            [
                {"type": "trajectory", "stage": "approach", "trajectory_file": "00_approach.npz"},
                {"type": "gripper", "closed": True},
            ]
        ),
        encoding="utf-8",
    )
    events = load_replay_events(manifest)
    assert events[0]["trajectory"].joint_names == ("j1", "j2")
    np.testing.assert_array_equal(events[0]["trajectory"].positions[-1], [2.0, 3.0])
    assert float(events[0]["trajectory"].dt) == 0.05
    assert events[1]["closed"] is True


@pytest.mark.parametrize("payload", [
    {"schema": "unknown", "commands": []},
    {"schema": "rm75.compiled_pickplace_program/v1", "commands": {}},
])
def test_load_replay_events_rejects_invalid_program_manifest(tmp_path, payload):
    path = tmp_path / "execution.json"
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError):
        load_replay_events(path)


def test_load_new_program_rejects_trajectory_path_escape(tmp_path):
    path = tmp_path / "execution.json"
    path.write_text(json.dumps({
        "schema": "rm75.compiled_pickplace_program/v1",
        "commands": [{"type": "trajectory", "trajectory_file": "../outside.npz"}],
    }))
    with pytest.raises(ValueError, match="escapes package"):
        load_replay_events(path)
