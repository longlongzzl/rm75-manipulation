"""Compile and continuously replay complete pick-place manipulation programs.

The existing :class:`PickPlaceCoordinator` remains the authority for candidate
screening, attachment-aware planning, and stage construction.  This module
replaces the physical executor with an in-memory recorder, allowing a complete
multi-atom program to be planned before the real robot starts moving.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol

import numpy as np

from rm75_app.orchestration.multi_object_executor import TaskSceneState
from rm75_app.pickplace.atom_task_builder import FixedSceneAtomTaskBuilder
from rm75_app.pickplace.coordinator import PickPlaceCoordinator, PickPlaceRunResult
from rm75_app.planning.contracts import JointConfiguration, JointTrajectory
from rm75_app.tasks.manipulation_plan import ManipulationAtom, ManipulationPlan

from .contracts import SceneStamp, stable_fingerprint


class TrajectorySink(Protocol):
    def execute_trajectory(self, stage: str, trajectory: JointTrajectory) -> None: ...

    def set_gripper(self, closed: bool) -> None: ...


@dataclass(frozen=True)
class AtomBoundaryCommand:
    atom_id: str
    starting: bool
    success: bool | None = None


@dataclass(frozen=True)
class TrajectoryCommand:
    atom_id: str
    stage: str
    trajectory: JointTrajectory


@dataclass(frozen=True)
class GripperCommand:
    atom_id: str
    closed: bool


PickPlaceCommand = AtomBoundaryCommand | TrajectoryCommand | GripperCommand


class InMemoryTrajectoryRecorder:
    """Executor-compatible recorder with strict inter-stage continuity checks."""

    def __init__(self, *, max_stage_start_gap_rad: float = 0.10) -> None:
        self.commands: list[PickPlaceCommand] = []
        self.current_atom_id: str | None = None
        self.max_stage_start_gap_rad = float(max_stage_start_gap_rad)
        self._last_end: np.ndarray | None = None
        self._last_joint_names: tuple[str, ...] | None = None

    def begin_atom(self, atom_id: str) -> None:
        if self.current_atom_id is not None:
            raise RuntimeError("cannot begin an atom while another atom is active")
        self.current_atom_id = str(atom_id)
        self.commands.append(AtomBoundaryCommand(str(atom_id), True))

    def end_atom(self, atom_id: str, *, success: bool) -> None:
        if self.current_atom_id != str(atom_id):
            raise RuntimeError("atom end does not match active atom")
        self.commands.append(AtomBoundaryCommand(str(atom_id), False, bool(success)))
        self.current_atom_id = None

    def execute_trajectory(self, stage: str, trajectory: JointTrajectory) -> None:
        if self.current_atom_id is None:
            raise RuntimeError("trajectory recorded outside an atom")
        names = tuple(trajectory.joint_names)
        start = np.asarray(trajectory.positions[0], dtype=np.float64)
        if self._last_end is not None:
            if names != self._last_joint_names:
                raise ValueError("joint names changed inside one compiled program")
            gap = float(np.max(np.abs(start - self._last_end)))
            if gap > self.max_stage_start_gap_rad:
                raise ValueError(
                    f"trajectory discontinuity before {stage!r}: {gap:.6f} rad "
                    f"> {self.max_stage_start_gap_rad:.6f} rad"
                )
        copied = JointTrajectory(
            names,
            np.asarray(trajectory.positions, dtype=np.float64).copy(),
            dt=(
                None
                if trajectory.dt is None
                else np.asarray(trajectory.dt, dtype=np.float64).copy()
                if isinstance(trajectory.dt, np.ndarray)
                else float(trajectory.dt)
            ),
        )
        self.commands.append(
            TrajectoryCommand(self.current_atom_id, str(stage), copied)
        )
        self._last_end = copied.positions[-1].copy()
        self._last_joint_names = names

    def set_gripper(self, closed: bool) -> None:
        if self.current_atom_id is None:
            raise RuntimeError("gripper command recorded outside an atom")
        self.commands.append(GripperCommand(self.current_atom_id, bool(closed)))


@dataclass(frozen=True)
class CompiledPickPlaceAtom:
    atom: ManipulationAtom
    commands: tuple[PickPlaceCommand, ...]
    selected_grasp: str | None
    selected_place: str | None
    source_scene_revision: int
    predicted_scene_revision: int
    planning_time_s: float
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "commands", tuple(self.commands))
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


@dataclass(frozen=True)
class CompiledPickPlaceProgram:
    plan_id: str
    source_stamp: SceneStamp
    atoms: tuple[CompiledPickPlaceAtom, ...]
    predicted_final_scene: TaskSceneState
    planning_time_s: float
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "atoms", tuple(self.atoms))
        object.__setattr__(self, "predicted_final_scene", self.predicted_final_scene.copy())
        object.__setattr__(self, "metadata", dict(self.metadata))

    @property
    def commands(self) -> tuple[PickPlaceCommand, ...]:
        return tuple(command for atom in self.atoms for command in atom.commands)

    @property
    def final_joint_configuration(self) -> JointConfiguration | None:
        scene = self.predicted_final_scene
        if not scene.joint_names or scene.joint_positions is None:
            return None
        return JointConfiguration(scene.joint_names, scene.joint_positions)

    def export(self, directory: str | Path) -> Path:
        """Export a planner-independent JSON/NPZ package."""

        root = Path(directory).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        manifest_commands: list[dict[str, Any]] = []
        trajectory_index = 0
        for command in self.commands:
            if isinstance(command, AtomBoundaryCommand):
                manifest_commands.append(
                    {
                        "type": "atom_start" if command.starting else "atom_end",
                        "atom_id": command.atom_id,
                        "success": command.success,
                    }
                )
            elif isinstance(command, GripperCommand):
                manifest_commands.append(
                    {
                        "type": "gripper",
                        "atom_id": command.atom_id,
                        "closed": command.closed,
                    }
                )
            else:
                filename = f"{trajectory_index:03d}_{command.atom_id}_{command.stage}.npz"
                filename = "".join(
                    char if char.isalnum() or char in "._-" else "_"
                    for char in filename
                )
                payload: dict[str, np.ndarray] = {
                    "positions": np.asarray(
                        command.trajectory.positions, dtype=np.float64
                    ),
                    "joint_names": np.asarray(
                        command.trajectory.joint_names, dtype=np.str_
                    ),
                }
                if command.trajectory.dt is not None:
                    payload["dt"] = np.asarray(
                        command.trajectory.dt, dtype=np.float64
                    )
                np.savez_compressed(root / filename, **payload)
                manifest_commands.append(
                    {
                        "type": "trajectory",
                        "atom_id": command.atom_id,
                        "stage": command.stage,
                        "trajectory_file": filename,
                        "points": int(len(command.trajectory.positions)),
                    }
                )
                trajectory_index += 1
        manifest = {
            "schema": "rm75.compiled_pickplace_program/v1",
            "plan_id": self.plan_id,
            "source_stamp": {
                "revision": self.source_stamp.revision,
                "backend_revision": self.source_stamp.backend_revision,
                "fingerprint": self.source_stamp.fingerprint,
            },
            "planning_time_s": self.planning_time_s,
            "atoms": [
                {
                    "atom_id": item.atom.atom_id,
                    "object_id": item.atom.object_id,
                    "selected_grasp": item.selected_grasp,
                    "selected_place": item.selected_place,
                    "source_scene_revision": item.source_scene_revision,
                    "predicted_scene_revision": item.predicted_scene_revision,
                    "planning_time_s": item.planning_time_s,
                    "diagnostics": dict(item.diagnostics),
                }
                for item in self.atoms
            ],
            "commands": manifest_commands,
            "predicted_final_scene": self.predicted_final_scene.as_dict(),
            "metadata": dict(self.metadata),
        }
        manifest_path = root / "execution.json"
        temporary = root / "execution.json.tmp"
        temporary.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary.replace(manifest_path)
        return manifest_path


@dataclass(frozen=True)
class PickPlaceCompilationResult:
    success: bool
    program: CompiledPickPlaceProgram | None
    failed_atom_id: str | None = None
    failure_stage: str | None = None
    message: str = ""
    partial_atoms: tuple[CompiledPickPlaceAtom, ...] = ()
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "partial_atoms", tuple(self.partial_atoms))
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


class PickPlaceProgramCompiler:
    """Compile a complete static-scene plan before motion starts."""

    def __init__(
        self,
        planner: Any,
        task_builder: FixedSceneAtomTaskBuilder,
        *,
        relation_screen_mode: str = "lazy_place",
        grasp_fallback_mode: str = "primary_only",
        max_stage_start_gap_rad: float = 0.10,
    ) -> None:
        self.planner = planner
        self.task_builder = task_builder
        self.relation_screen_mode = str(relation_screen_mode)
        self.grasp_fallback_mode = str(grasp_fallback_mode)
        self.max_stage_start_gap_rad = float(max_stage_start_gap_rad)

    def compile(
        self,
        plan: ManipulationPlan,
        scene: TaskSceneState,
    ) -> PickPlaceCompilationResult:
        predicted = scene.copy()
        source_stamp = SceneStamp(
            revision=int(scene.revision),
            backend_revision=scene.backend_revision,
            fingerprint=stable_fingerprint(scene.as_dict()),
        )
        recorder = InMemoryTrajectoryRecorder(
            max_stage_start_gap_rad=self.max_stage_start_gap_rad
        )
        compiled_atoms: list[CompiledPickPlaceAtom] = []
        total_started = time.perf_counter()
        for atom in plan.atoms:
            source_revision = int(predicted.revision)
            command_start = len(recorder.commands)
            recorder.begin_atom(atom.atom_id)
            atom_started = time.perf_counter()
            try:
                task = self.task_builder(atom, predicted)
                task_overrides: dict[str, Any] = {}
                numeric_overrides = {
                    "place_clearance_m": "place_clearance",
                    "grasp_approach_offset_m": "grasp_approach_offset",
                    "lift_height_m": "lift_height",
                }
                for metadata_key, task_field in numeric_overrides.items():
                    if metadata_key in atom.metadata:
                        task_overrides[task_field] = float(
                            atom.metadata[metadata_key]
                        )
                if "max_motion_candidates" in atom.metadata:
                    task_overrides["max_motion_candidates"] = int(
                        atom.metadata["max_motion_candidates"]
                    )
                if task_overrides:
                    task = replace(task, **task_overrides)
                coordinator = PickPlaceCoordinator(
                    self.planner,
                    recorder,
                    relation_screen_mode=self.relation_screen_mode,
                    grasp_fallback_mode=self.grasp_fallback_mode,
                )
                result: PickPlaceRunResult = coordinator.run(task)
            except Exception as exc:
                recorder.end_atom(atom.atom_id, success=False)
                return PickPlaceCompilationResult(
                    False,
                    None,
                    failed_atom_id=atom.atom_id,
                    failure_stage="exception",
                    message=f"{type(exc).__name__}: {exc}",
                    partial_atoms=tuple(compiled_atoms),
                    diagnostics={"exception_type": type(exc).__name__},
                )
            recorder.end_atom(atom.atom_id, success=result.success)
            elapsed = time.perf_counter() - atom_started
            if not result.success:
                return PickPlaceCompilationResult(
                    False,
                    None,
                    failed_atom_id=atom.atom_id,
                    failure_stage=result.failure_stage,
                    message=result.message or "pick-place planning failed",
                    partial_atoms=tuple(compiled_atoms),
                    diagnostics=dict(result.diagnostics),
                )
            if not result.stages:
                return PickPlaceCompilationResult(
                    False,
                    None,
                    failed_atom_id=atom.atom_id,
                    failure_stage="empty_program",
                    message="successful coordinator result contains no stages",
                    partial_atoms=tuple(compiled_atoms),
                )
            final_joint = result.stages[-1].end
            predicted.set_joints(final_joint.names, final_joint.positions)
            predicted.commit_object_pose(atom.object_id, atom.target_pose)
            atom_commands = tuple(recorder.commands[command_start:])
            compiled_atoms.append(
                CompiledPickPlaceAtom(
                    atom,
                    atom_commands,
                    result.selected_grasp,
                    result.selected_place,
                    source_revision,
                    int(predicted.revision),
                    elapsed,
                    diagnostics=dict(result.diagnostics),
                )
            )
        program = CompiledPickPlaceProgram(
            plan.plan_id,
            source_stamp,
            tuple(compiled_atoms),
            predicted,
            time.perf_counter() - total_started,
            metadata={
                "source_scene_file": plan.scene_file,
                "user_command": plan.user_command,
                "relation_screen_mode": self.relation_screen_mode,
                "grasp_fallback_mode": self.grasp_fallback_mode,
            },
        )
        return PickPlaceCompilationResult(True, program)


@dataclass(frozen=True)
class PickPlaceExecutionReport:
    success: bool
    completed_atoms: tuple[str, ...]
    failed_atom_id: str | None = None
    failed_stage: str | None = None
    message: str = ""
    total_time_s: float = 0.0
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "completed_atoms", tuple(self.completed_atoms))
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


JointStateProvider = Callable[[], JointConfiguration]
AtomCallback = Callable[[str, bool], None]


class PickPlaceProgramExecutor:
    """Replay a precompiled program without planner pauses between stages."""

    def __init__(
        self,
        sink: TrajectorySink,
        *,
        joint_state_provider: JointStateProvider | None = None,
        max_initial_joint_gap_rad: float = 0.12,
        atom_callback: AtomCallback | None = None,
    ) -> None:
        self.sink = sink
        self.joint_state_provider = joint_state_provider
        self.max_initial_joint_gap_rad = float(max_initial_joint_gap_rad)
        self.atom_callback = atom_callback

    def _validate_initial_joint_state(
        self,
        program: CompiledPickPlaceProgram,
    ) -> tuple[bool, str, float | None]:
        first = next(
            (
                item
                for item in program.commands
                if isinstance(item, TrajectoryCommand)
            ),
            None,
        )
        if first is None or self.joint_state_provider is None:
            return True, "", None
        observed = self.joint_state_provider()
        if tuple(observed.names) != tuple(first.trajectory.joint_names):
            return False, "real joint names do not match the compiled program", None
        gap = float(
            np.max(
                np.abs(
                    np.asarray(observed.positions, dtype=np.float64)
                    - np.asarray(first.trajectory.positions[0], dtype=np.float64)
                )
            )
        )
        if gap > self.max_initial_joint_gap_rad:
            return (
                False,
                f"initial joint gap {gap:.6f} rad exceeds "
                f"{self.max_initial_joint_gap_rad:.6f} rad",
                gap,
            )
        return True, "", gap

    def execute(
        self,
        program: CompiledPickPlaceProgram,
    ) -> PickPlaceExecutionReport:
        valid, message, initial_gap = self._validate_initial_joint_state(program)
        if not valid:
            return PickPlaceExecutionReport(
                False,
                (),
                failed_stage="initial_state_validation",
                message=message,
                diagnostics={"initial_joint_gap_rad": initial_gap},
            )
        completed: list[str] = []
        active_atom: str | None = None
        active_stage: str | None = None
        started = time.perf_counter()
        try:
            for command in program.commands:
                if isinstance(command, AtomBoundaryCommand):
                    if command.starting:
                        active_atom = command.atom_id
                        if self.atom_callback is not None:
                            self.atom_callback(command.atom_id, True)
                    else:
                        if command.success:
                            completed.append(command.atom_id)
                        if self.atom_callback is not None:
                            self.atom_callback(command.atom_id, False)
                        active_atom = None
                    continue
                if isinstance(command, GripperCommand):
                    active_stage = "gripper_close" if command.closed else "gripper_open"
                    self.sink.set_gripper(command.closed)
                    continue
                active_stage = command.stage
                self.sink.execute_trajectory(command.stage, command.trajectory)
        except Exception as exc:
            return PickPlaceExecutionReport(
                False,
                tuple(completed),
                failed_atom_id=active_atom,
                failed_stage=active_stage,
                message=f"{type(exc).__name__}: {exc}",
                total_time_s=time.perf_counter() - started,
                diagnostics={"initial_joint_gap_rad": initial_gap},
            )
        return PickPlaceExecutionReport(
            True,
            tuple(completed),
            total_time_s=time.perf_counter() - started,
            diagnostics={"initial_joint_gap_rad": initial_gap},
        )
