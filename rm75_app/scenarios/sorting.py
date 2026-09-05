"""Frontend and end-to-end service for multi-object tabletop sorting."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from rm75_app.orchestration.multi_object_executor import TaskSceneState
from rm75_app.tasks.manipulation_plan import (
    ManipulationAtom,
    ManipulationPlan,
    ManipulationPrimitive,
    PlacementMode,
    SuccessCriteria,
)

from .pickplace_program import (
    PickPlaceCompilationResult,
    PickPlaceExecutionReport,
    PickPlaceProgramCompiler,
    PickPlaceProgramExecutor,
)


def _pose(value: Sequence[Sequence[float]]) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError("target pose must be a finite 4x4 matrix")
    if not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError("target pose is not homogeneous")
    return result.copy()


@dataclass(frozen=True)
class SortingTarget:
    target_id: str
    pose: Sequence[Sequence[float]]
    support_object_id: str | None = None
    capacity: int = 1
    slot_spacing_m: float = 0.07
    placement_mode: PlacementMode = PlacementMode.SURFACE
    metadata: Mapping[str, Any] = field(default_factory=dict)

    success_relation: str = "target_pose"

    def __post_init__(self) -> None:
        if not self.target_id:
            raise ValueError("target_id must not be empty")
        if self.success_relation not in {"target_pose", "inside"}:
            raise ValueError("unsupported sorting success_relation")
        if self.success_relation == "inside" and not self.support_object_id:
            raise ValueError("inside success_relation requires support_object_id")
        if self.capacity < 1:
            raise ValueError("target capacity must be positive")
        if self.slot_spacing_m <= 0.0:
            raise ValueError("slot spacing must be positive")
        object.__setattr__(self, "pose", _pose(self.pose))
        object.__setattr__(self, "placement_mode", PlacementMode(self.placement_mode))
        object.__setattr__(self, "metadata", dict(self.metadata))

    def pose_for_slot(self, index: int) -> np.ndarray:
        if index < 0 or index >= self.capacity:
            raise IndexError(index)
        output = np.asarray(self.pose, dtype=np.float64).copy()
        if self.capacity == 1:
            return output
        columns = int(np.ceil(np.sqrt(self.capacity)))
        rows = int(np.ceil(self.capacity / columns))
        row, column = divmod(index, columns)
        local_offset = np.asarray(
            [
                (column - 0.5 * (columns - 1)) * self.slot_spacing_m,
                (row - 0.5 * (rows - 1)) * self.slot_spacing_m,
                0.0,
            ],
            dtype=np.float64,
        )
        output[:3, 3] += output[:3, :3] @ local_offset
        return output


@dataclass(frozen=True)
class SortingAssignment:
    object_id: str
    target_id: str
    priority: int = 0
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.object_id or not self.target_id:
            raise ValueError("sorting object and target ids must not be empty")
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class SortingRequest:
    request_id: str
    scene_file: str
    assignments: tuple[SortingAssignment, ...]
    targets: tuple[SortingTarget, ...]
    user_command: str = ""
    buffer_target_id: str | None = None
    occupancy_radius_m: float = 0.045
    position_tolerance_m: float = 0.02
    orientation_tolerance_deg: float = 20.0
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.request_id or not self.scene_file:
            raise ValueError("request_id and scene_file must not be empty")
        assignments = tuple(self.assignments)
        targets = tuple(self.targets)
        if not assignments:
            raise ValueError("sorting request must contain at least one assignment")
        object_ids = [item.object_id for item in assignments]
        if len(set(object_ids)) != len(object_ids):
            raise ValueError("an object may have only one final sorting assignment")
        target_ids = [item.target_id for item in targets]
        if len(set(target_ids)) != len(target_ids):
            raise ValueError("sorting target ids must be unique")
        target_by_id = {item.target_id: item for item in targets}
        unknown = {item.target_id for item in assignments} - set(target_by_id)
        if unknown:
            raise ValueError(f"unknown sorting targets: {sorted(unknown)}")
        counts = {
            target_id: sum(item.target_id == target_id for item in assignments)
            for target_id in target_by_id
        }
        overfull = {
            target_id: count
            for target_id, count in counts.items()
            if count > target_by_id[target_id].capacity
        }
        if overfull:
            raise ValueError(f"sorting target capacity exceeded: {overfull}")
        if self.buffer_target_id is not None and self.buffer_target_id not in target_by_id:
            raise ValueError("buffer_target_id is not a declared target")
        if self.occupancy_radius_m <= 0.0:
            raise ValueError("occupancy radius must be positive")
        object.__setattr__(self, "assignments", assignments)
        object.__setattr__(self, "targets", targets)
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class _Move:
    object_id: str
    target_id: str
    target_pose: np.ndarray
    support_object_id: str | None
    placement_mode: PlacementMode
    priority: int
    temporary: bool = False
    depends_on_objects: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


class SortingPlanCompiler:
    """Compile assignments into collision-aware pick-place atoms.

    A move into a location occupied by another assigned object depends on that
    object's move. Cycles are rejected unless a buffer target is declared; one
    object is then moved to the buffer before the remaining cycle is cleared.
    """

    def _slot_poses(
        self,
        request: SortingRequest,
    ) -> dict[str, tuple[SortingTarget, np.ndarray]]:
        by_target = {item.target_id: item for item in request.targets}
        counters = {item.target_id: 0 for item in request.targets}
        output: dict[str, tuple[SortingTarget, np.ndarray]] = {}
        for assignment in sorted(
            request.assignments,
            key=lambda item: (-item.priority, item.object_id),
        ):
            target = by_target[assignment.target_id]
            slot = counters[target.target_id]
            counters[target.target_id] += 1
            output[assignment.object_id] = (target, target.pose_for_slot(slot))
        return output

    @staticmethod
    def _nearest_assigned_occupant(
        object_id: str,
        target_pose: np.ndarray,
        scene: TaskSceneState,
        assigned: set[str],
        radius: float,
    ) -> str | None:
        center = target_pose[:3, 3]
        candidates = []
        for other_id in assigned:
            if other_id == object_id or other_id not in scene.objects:
                continue
            position = scene.objects[other_id].pose[:3, 3]
            distance = float(np.linalg.norm(position - center))
            if distance <= radius:
                candidates.append((distance, other_id))
        return None if not candidates else min(candidates)[1]

    @staticmethod
    def _find_cycle(dependency: Mapping[str, str | None]) -> tuple[str, ...]:
        visited: set[str] = set()
        for start in dependency:
            if start in visited:
                continue
            order: list[str] = []
            index: dict[str, int] = {}
            current: str | None = start
            while current is not None and current in dependency:
                if current in index:
                    return tuple(order[index[current] :])
                if current in visited:
                    break
                index[current] = len(order)
                order.append(current)
                current = dependency.get(current)
            visited.update(order)
        return ()

    def _moves(
        self,
        request: SortingRequest,
        scene: TaskSceneState,
    ) -> tuple[_Move, ...]:
        assigned = {item.object_id for item in request.assignments}
        missing = assigned - set(scene.objects)
        if missing:
            raise KeyError(f"sorting objects missing from scene: {sorted(missing)}")
        slots = self._slot_poses(request)
        assignment_by_object = {
            item.object_id: item for item in request.assignments
        }
        dependency: dict[str, str | None] = {}
        for object_id, (_target, pose) in slots.items():
            dependency[object_id] = self._nearest_assigned_occupant(
                object_id,
                pose,
                scene,
                assigned,
                request.occupancy_radius_m,
            )

        cycle = self._find_cycle(dependency)
        moves: list[_Move] = []
        buffered: str | None = None
        released_by_buffer: set[str] = set()
        if cycle:
            if request.buffer_target_id is None:
                raise ValueError(
                    "sorting target occupancy contains a cycle; declare a buffer target"
                )
            pivot = max(
                cycle,
                key=lambda item: (
                    assignment_by_object[item].priority,
                    item,
                ),
            )
            buffer_target = next(
                item
                for item in request.targets
                if item.target_id == request.buffer_target_id
            )
            if assignment_by_object[pivot].target_id == buffer_target.target_id:
                raise ValueError("buffer target is also the pivot's final target")
            moves.append(
                _Move(
                    pivot,
                    buffer_target.target_id,
                    buffer_target.pose_for_slot(0),
                    buffer_target.support_object_id,
                    buffer_target.placement_mode,
                    assignment_by_object[pivot].priority,
                    temporary=True,
                    metadata={"cycle": list(cycle), "cycle_break": True},
                )
            )
            dependency[pivot] = None
            for key, value in list(dependency.items()):
                if value == pivot:
                    dependency[key] = None
                    released_by_buffer.add(key)
            buffered = pivot

        pending = set(assigned)
        if buffered is not None:
            pending.remove(buffered)
        emitted: set[str] = set()
        while pending:
            ready = [
                object_id
                for object_id in pending
                if dependency.get(object_id) is None
                or dependency[object_id] in emitted
                or dependency[object_id] not in pending
            ]
            if not ready:
                remaining_cycle = self._find_cycle(
                    {item: dependency.get(item) for item in pending}
                )
                raise ValueError(
                    "sorting dependency cycle remains after buffer expansion: "
                    f"{list(remaining_cycle)}"
                )
            ready.sort(
                key=lambda item: (
                    -assignment_by_object[item].priority,
                    float(
                        np.linalg.norm(
                            scene.objects[item].pose[:3, 3]
                            - slots[item][1][:3, 3]
                        )
                    ),
                    item,
                )
            )
            for object_id in ready:
                assignment = assignment_by_object[object_id]
                target, pose = slots[object_id]
                dependency_object = dependency.get(object_id)
                buffer_dependency = (
                    (buffered,)
                    if buffered is not None and object_id in released_by_buffer
                    else ()
                )
                moves.append(
                    _Move(
                        object_id,
                        target.target_id,
                        pose,
                        target.support_object_id,
                        target.placement_mode,
                        assignment.priority,
                        depends_on_objects=(
                            buffer_dependency
                            if dependency_object is None
                            else tuple(
                                dict.fromkeys(
                                    (*buffer_dependency, dependency_object)
                                )
                            )
                        ),
                        metadata=dict(assignment.metadata),
                    )
                )
                pending.remove(object_id)
                emitted.add(object_id)

        if buffered is not None:
            assignment = assignment_by_object[buffered]
            target, pose = slots[buffered]
            moves.append(
                _Move(
                    buffered,
                    target.target_id,
                    pose,
                    target.support_object_id,
                    target.placement_mode,
                    assignment.priority,
                    depends_on_objects=tuple(
                        object_id for object_id in cycle if object_id != buffered
                    ),
                    metadata={
                        **dict(assignment.metadata),
                        "cycle_return_from_buffer": True,
                    },
                )
            )
        return tuple(moves)

    def compile(
        self,
        request: SortingRequest,
        scene: TaskSceneState,
    ) -> ManipulationPlan:
        targets = {target.target_id: target for target in request.targets}
        for target in request.targets:
            if target.success_relation == "inside" and target.support_object_id not in scene.objects:
                raise ValueError(f"inside support {target.support_object_id!r} is absent from scene")
        for assignment in request.assignments:
            target = targets[assignment.target_id]
            if target.success_relation == "inside" and target.support_object_id == assignment.object_id:
                raise ValueError("object cannot be placed inside itself")
        moves = self._moves(request, scene)
        last_atom_for_object: dict[str, str] = {}
        atoms: list[ManipulationAtom] = []
        for index, move in enumerate(moves, start=1):
            state = scene.objects[move.object_id]
            atom_id = f"sort_{index:02d}_{move.object_id}"
            dependency_atoms = [
                last_atom_for_object[item]
                for item in move.depends_on_objects
                if item in last_atom_for_object
            ]
            if move.object_id in last_atom_for_object:
                dependency_atoms.append(last_atom_for_object[move.object_id])
            atom = ManipulationAtom(
                atom_id=atom_id,
                primitive=ManipulationPrimitive.PICK_PLACE,
                object_id=move.object_id,
                object_asset=state.asset_name,
                target_pose=move.target_pose,
                support_object_id=move.support_object_id,
                placement_mode=move.placement_mode,
                semantic_operator=(
                    "sorting_buffer" if move.temporary else
                    "inside" if targets[move.target_id].success_relation == "inside" else "sorting_target"
                ),
                success=SuccessCriteria(
                    targets[move.target_id].success_relation,
                    request.position_tolerance_m,
                    request.orientation_tolerance_deg,
                    20,
                ),
                depends_on=tuple(dict.fromkeys(dependency_atoms)),
                metadata={
                    "sorting_target_id": move.target_id,
                    "sorting_priority": move.priority,
                    "temporary_buffer_move": move.temporary,
                    **dict(move.metadata),
                },
            )
            atoms.append(atom)
            last_atom_for_object[move.object_id] = atom_id
        return ManipulationPlan(
            plan_id=request.request_id,
            scene_file=request.scene_file,
            atoms=tuple(atoms),
            user_command=request.user_command,
            metadata={
                "scenario": "sorting",
                "compile_full_program_before_execution": True,
                **dict(request.metadata),
            },
        )


@dataclass(frozen=True)
class PreparedSortingProgram:
    manipulation_plan: ManipulationPlan
    compilation: PickPlaceCompilationResult


class SortingSystem:
    """High-level facade for frontend compilation, planning, and replay."""

    def __init__(
        self,
        plan_compiler: SortingPlanCompiler,
        motion_compiler: PickPlaceProgramCompiler,
        program_executor: PickPlaceProgramExecutor | None = None,
    ) -> None:
        self.plan_compiler = plan_compiler
        self.motion_compiler = motion_compiler
        self.program_executor = program_executor

    def prepare(
        self,
        request: SortingRequest,
        scene: TaskSceneState,
    ) -> PreparedSortingProgram:
        plan = self.plan_compiler.compile(request, scene)
        return PreparedSortingProgram(
            plan,
            self.motion_compiler.compile(plan, scene),
        )

    def execute(
        self,
        prepared: PreparedSortingProgram,
    ) -> PickPlaceExecutionReport:
        if not prepared.compilation.success or prepared.compilation.program is None:
            return PickPlaceExecutionReport(
                False,
                (),
                failed_atom_id=prepared.compilation.failed_atom_id,
                failed_stage=prepared.compilation.failure_stage,
                message=prepared.compilation.message,
                diagnostics=prepared.compilation.diagnostics,
            )
        if self.program_executor is None:
            raise RuntimeError("sorting system has no physical/simulation executor")
        return self.program_executor.execute(prepared.compilation.program)
