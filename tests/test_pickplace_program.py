from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np

from rm75_app.orchestration.multi_object_executor import (
    SceneObjectState,
    TaskSceneState,
)
from rm75_app.pickplace.coordinator import ExecutedStage, PickPlaceRunResult
from rm75_app.planning.contracts import JointConfiguration, JointTrajectory
from rm75_app.scenarios import pickplace_program as program_module
from rm75_app.scenarios.pickplace_program import (
    AtomBoundaryCommand,
    GripperCommand,
    InMemoryTrajectoryRecorder,
    PickPlaceProgramCompiler,
    PickPlaceProgramExecutor,
    TrajectoryCommand,
)
from rm75_app.tasks.manipulation_plan import (
    ManipulationAtom,
    ManipulationPlan,
    ManipulationPrimitive,
)


def _pose(x: float) -> np.ndarray:
    value = np.eye(4)
    value[0, 3] = x
    return value


def test_recorder_rejects_inter_stage_joint_discontinuity() -> None:
    recorder = InMemoryTrajectoryRecorder(max_stage_start_gap_rad=0.1)
    recorder.begin_atom("a")
    recorder.execute_trajectory(
        "one",
        JointTrajectory(("j",), np.asarray([[0.0], [0.2]])),
    )

    try:
        recorder.execute_trajectory(
            "two",
            JointTrajectory(("j",), np.asarray([[0.5], [0.6]])),
        )
    except ValueError as exc:
        assert "discontinuity" in str(exc)
    else:
        raise AssertionError("discontinuous stages were accepted")


def test_compiler_builds_all_atoms_before_continuous_replay(
    monkeypatch,
    tmp_path: Path,
) -> None:
    joints = ("j1", "j2")
    scene = TaskSceneState(
        {
            "a": SceneObjectState("a", "redcube", _pose(0.0)),
            "b": SceneObjectState("b", "lvmukuai", _pose(0.1)),
        },
        joint_names=joints,
        joint_positions=np.asarray([0.0, 0.0]),
    )
    atoms = (
        ManipulationAtom(
            "a1",
            ManipulationPrimitive.PICK_PLACE,
            "a",
            "redcube",
            _pose(0.2),
        ),
        ManipulationAtom(
            "a2",
            ManipulationPrimitive.PICK_PLACE,
            "b",
            "lvmukuai",
            _pose(0.3),
            depends_on=("a1",),
        ),
    )
    plan = ManipulationPlan("plan", "scene.json", atoms)
    build_currents: list[np.ndarray] = []

    def builder(atom, current_scene):
        del atom
        build_currents.append(current_scene.joint_positions.copy())
        return SimpleNamespace(
            current=JointConfiguration(
                current_scene.joint_names,
                current_scene.joint_positions,
            )
        )

    class FakeCoordinator:
        def __init__(self, planner, executor, **kwargs):
            del planner, kwargs
            self.executor = executor

        def run(self, task):
            start = np.asarray(task.current.positions, dtype=np.float64)
            end = start + np.asarray([0.1, 0.05])
            trajectory = JointTrajectory(joints, np.stack((start, end)))
            self.executor.execute_trajectory("approach", trajectory)
            self.executor.set_gripper(True)
            self.executor.set_gripper(False)
            configuration = JointConfiguration(joints, end)
            return PickPlaceRunResult(
                True,
                (ExecutedStage("approach", "candidate", configuration),),
                selected_grasp="g",
                selected_place="p",
            )

    monkeypatch.setattr(program_module, "PickPlaceCoordinator", FakeCoordinator)
    compilation = PickPlaceProgramCompiler(object(), builder).compile(plan, scene)

    assert compilation.success
    assert compilation.program is not None
    assert len(compilation.program.atoms) == 2
    assert np.allclose(build_currents[0], [0.0, 0.0])
    assert np.allclose(build_currents[1], [0.1, 0.05])
    assert np.allclose(
        compilation.program.predicted_final_scene.objects["a"].pose,
        _pose(0.2),
    )
    assert np.allclose(
        compilation.program.predicted_final_scene.objects["b"].pose,
        _pose(0.3),
    )
    assert sum(
        isinstance(item, TrajectoryCommand)
        for item in compilation.program.commands
    ) == 2

    manifest = compilation.program.export(tmp_path / "compiled")
    assert manifest.exists()
    assert len(list(manifest.parent.glob("*.npz"))) == 2

    events: list[tuple[str, object]] = []

    class Sink:
        def execute_trajectory(self, stage, trajectory):
            events.append(("trajectory", stage))
            assert trajectory.positions.shape == (2, 2)

        def set_gripper(self, closed):
            events.append(("gripper", bool(closed)))

    executor = PickPlaceProgramExecutor(
        Sink(),
        joint_state_provider=lambda: JointConfiguration(joints, [0.0, 0.0]),
    )
    report = executor.execute(compilation.program)

    assert report.success
    assert report.completed_atoms == ("a1", "a2")
    assert events == [
        ("trajectory", "approach"),
        ("gripper", True),
        ("gripper", False),
        ("trajectory", "approach"),
        ("gripper", True),
        ("gripper", False),
    ]


def test_executor_refuses_program_from_wrong_initial_joint_state() -> None:
    recorder = InMemoryTrajectoryRecorder()
    recorder.begin_atom("a")
    recorder.execute_trajectory(
        "move",
        JointTrajectory(("j",), np.asarray([[0.0], [0.1]])),
    )
    recorder.end_atom("a", success=True)
    atom = ManipulationAtom(
        "a",
        ManipulationPrimitive.PICK_PLACE,
        "object",
        "redcube",
        _pose(0.1),
    )
    scene = TaskSceneState(
        {"object": SceneObjectState("object", "redcube", _pose(0.1))},
        joint_names=("j",),
        joint_positions=np.asarray([0.1]),
    )
    compiled_atom = program_module.CompiledPickPlaceAtom(
        atom,
        tuple(recorder.commands),
        "g",
        "p",
        0,
        1,
        0.1,
    )
    program = program_module.CompiledPickPlaceProgram(
        "p",
        program_module.SceneStamp(0),
        (compiled_atom,),
        scene,
        0.1,
    )

    class Sink:
        def execute_trajectory(self, stage, trajectory):
            raise AssertionError("stale program should not execute")

        def set_gripper(self, closed):
            raise AssertionError("stale program should not execute")

    report = PickPlaceProgramExecutor(
        Sink(),
        joint_state_provider=lambda: JointConfiguration(("j",), [0.4]),
        max_initial_joint_gap_rad=0.1,
    ).execute(program)

    assert not report.success
    assert report.failed_stage == "initial_state_validation"
