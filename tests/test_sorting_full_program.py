"""Offline integration: real frontend, task builder and coordinator; fake IK.

These tests establish software wiring, never geometric feasibility or dynamics.
"""
import json

import numpy as np
import pytest

from test_pickplace_coordinator import FakePlanner
from test_sorting_scenario import _swap_request, _swap_scene
from rm75_app.pickplace.atom_task_builder import FixedSceneAtomTaskBuilder
from rm75_app.planning.contracts import JointConfiguration
from rm75_app.scenarios.pickplace_program import TrajectoryCommand, GripperCommand
from rm75_app.scenarios.system import UnifiedManipulationSystem


@pytest.mark.parametrize("fail_atom", [None, 2])
def test_sorting_real_compilers_buffer_swap(tmp_path, fail_atom):
    scene = _swap_scene()
    original = scene.as_dict()
    planner = FakePlanner()
    builder = FixedSceneAtomTaskBuilder()
    built = []
    submitted = []

    def audited_builder(atom, predicted):
        built.append((atom, predicted.copy()))
        if len(built) == fail_atom:
            raise RuntimeError("injected second-atom build failure")
        return builder(atom, predicted)

    class Sink:
        def execute_trajectory(self, stage, trajectory):
            assert len(built) == 3
            submitted.append((stage, trajectory))

        def set_gripper(self, closed):
            assert len(built) == 3
            submitted.append(("gripper", closed))

    system = UnifiedManipulationSystem(
        planner, audited_builder, trajectory_sink=Sink(),
        joint_state_provider=lambda: JointConfiguration(
            scene.joint_names, scene.joint_positions),
    )
    prepared = system.prepare_sorting(_swap_request(with_buffer=True), scene)
    assert not submitted
    assert scene.as_dict() == original
    if fail_atom is not None:
        assert not prepared.compilation.success
        assert prepared.compilation.program is None
        assert len(prepared.compilation.partial_atoms) == 1
        assert prepared.compilation.failure_stage == "exception"
        assert "injected second-atom" in prepared.compilation.message
        assert not system.execute_sorting(prepared).success
        assert not submitted
        return

    assert prepared.compilation.success, prepared.compilation.message
    program = prepared.compilation.program
    assert len(program.atoms) == 3
    assert [atom.object_id for atom, _ in built] == ["b", "a", "b"]
    for index, compiled in enumerate(program.atoms):
        assert compiled.source_scene_revision == index
        assert compiled.predicted_scene_revision == index + 1
        trajectories = [c for c in compiled.commands if isinstance(c, TrajectoryCommand)]
        assert trajectories
        assert [c.closed for c in compiled.commands if isinstance(c, GripperCommand)] == [True, False]
        np.testing.assert_allclose(trajectories[0].trajectory.positions[0], built[index][1].joint_positions)
        if index:
            previous = program.atoms[index - 1].atom
            np.testing.assert_allclose(built[index][1].objects[previous.object_id].pose, previous.target_pose)
    trajectories = [c.trajectory for c in program.commands if isinstance(c, TrajectoryCommand)]
    for previous, following in zip(trajectories, trajectories[1:]):
        assert previous.joint_names == following.joint_names
        np.testing.assert_allclose(previous.positions[-1], following.positions[0])
    manifest = json.loads(program.export(tmp_path).read_text())
    exported = [c for c in manifest["commands"] if c["type"] == "trajectory"]
    assert len(exported) == len(trajectories)
    for entry, trajectory in zip(exported, trajectories):
        with np.load(tmp_path / entry["trajectory_file"], allow_pickle=False) as saved:
            np.testing.assert_array_equal(saved["positions"], trajectory.positions)
            assert tuple(saved["joint_names"]) == trajectory.joint_names
    planning_event_count = len(planner.events)
    assert system.execute_sorting(prepared).success
    assert len(planner.events) == planning_event_count
    assert len(submitted) == len(trajectories) + 6


def test_invalid_sorting_frontend_never_calls_motion_builder():
    def forbidden(*args):
        raise AssertionError("invalid symbolic plan reached motion planning")

    system = UnifiedManipulationSystem(object(), forbidden)
    with pytest.raises(ValueError, match="cycle"):
        system.prepare_sorting(_swap_request(with_buffer=False), _swap_scene())
