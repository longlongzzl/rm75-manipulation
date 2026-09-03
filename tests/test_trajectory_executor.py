from __future__ import annotations

import json

import numpy as np
import pytest

from rm75_app.execution.trajectory_executor import (
    ManiSkillTrajectoryExecutor,
    RecordingTrajectoryExecutor,
    downsample_joint_path,
)
from rm75_app.planning.contracts import JointTrajectory


def test_downsample_preserves_endpoints() -> None:
    path = np.arange(200, dtype=np.float64).reshape(100, 2)
    sampled = downsample_joint_path(path, 9)
    assert len(sampled) == 9
    np.testing.assert_array_equal(sampled[0], path[0])
    np.testing.assert_array_equal(sampled[-1], path[-1])


def test_recording_executor_writes_replay_artifact(tmp_path) -> None:
    output = tmp_path / "execution.json"
    executor = RecordingTrajectoryExecutor(output)
    executor.begin_atom("atom_01")
    executor.execute_trajectory(
        "lift", JointTrajectory(("j1", "j2"), np.asarray([[0.0, 0.0], [1.0, 2.0]]))
    )
    executor.set_gripper(True)
    executor.end_atom("atom_01", success=True)
    saved = json.loads(output.read_text(encoding="utf-8"))
    assert saved[0] == {"type": "atom_start", "atom_id": "atom_01"}
    assert saved[1]["stage"] == "lift"
    assert saved[1]["end"] == [1.0, 2.0]
    trajectory_path = tmp_path / saved[1]["trajectory_file"]
    with np.load(trajectory_path) as trajectory:
        np.testing.assert_array_equal(trajectory["positions"], [[0.0, 0.0], [1.0, 2.0]])
        assert trajectory["joint_names"].tolist() == ["j1", "j2"]
    assert saved[2] == {"type": "gripper", "closed": True}
    assert saved[3] == {"type": "atom_end", "atom_id": "atom_01", "success": True}


def test_recording_executor_rejects_discontinuous_stage_package(tmp_path) -> None:
    executor = RecordingTrajectoryExecutor(tmp_path / "execution.json")
    executor.execute_trajectory(
        "preplace", JointTrajectory(("j1", "j2"), np.asarray([[0.0, 0.0], [0.2, 0.3]]))
    )

    with pytest.raises(ValueError, match="trajectory discontinuity before stage 'place'"):
        executor.execute_trajectory(
            "place", JointTrajectory(("j1", "j2"), np.asarray([[2.0, -1.0], [2.1, -1.1]]))
        )


def test_maniskill_rm75_gripper_uses_negative_open_positive_closed() -> None:
    class FakeDemo:
        def __init__(self):
            self.gripper_commands = []

        def hold_current_and_set_gripper(self, value, steps):
            self.gripper_commands.append((value, steps))

    demo = FakeDemo()
    executor = ManiSkillTrajectoryExecutor(demo, gripper_steps=7)

    assert executor.gripper_open == -1.0
    assert executor.gripper_closed == 1.0
    executor.set_gripper(True)
    executor.set_gripper(False)

    assert demo.gripper_commands == [(1.0, 7), (-1.0, 7)]


def test_maniskill_executor_aggregates_transient_robot_contacts() -> None:
    class FakeDemo:
        def __init__(self):
            self.step = 0

        def compose_action(self, position, gripper_value):
            del gripper_value
            return np.asarray(position)

        def step_and_render(self, action, tag):
            del action, tag
            self.step += 1

        def robot_contact_pairs(self):
            if self.step == 1:
                return [{"body_a": "left_pad", "body_b": "bitong", "impulse_ns": 0.2}]
            if self.step == 2:
                return [{"body_a": "bitong", "body_b": "left_pad", "impulse_ns": 0.5}]
            return []

    executor = ManiSkillTrajectoryExecutor(FakeDemo())
    executor.execute_trajectory(
        "grasp",
        JointTrajectory(("j1",), np.asarray([[0.0], [0.1], [0.2]], dtype=np.float64)),
    )

    assert executor.last_contact_summary == [
        {
            "body_a": "bitong",
            "body_b": "left_pad",
            "max_impulse_ns": 0.5,
            "contact_steps": 2,
        }
    ]
