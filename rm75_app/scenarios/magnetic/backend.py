"""Magnetic-specific adapters on top of the shared PickPlace foundation.

The generic PickPlace contract historically exposed one contact support.  A
vertical magnetic panel inserted between two lying panels intentionally enters
the activation margin of both supports.  This adapter carries all support
instance ids on the place candidates and scopes the multi-support exemption to
endpoint/contact planning only; free-space transport still sees every support.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Iterable

from rm75_app.orchestration.multi_object_executor import TaskSceneState
from rm75_app.pickplace.atom_task_builder import FixedSceneAtomTaskBuilder
from rm75_app.pickplace.coordinator import PickPlaceTask
from rm75_app.planning.contracts import (
    BatchPlanningRequest,
    PlanningScene,
    PoseCandidate,
)
from rm75_app.tasks.manipulation_plan import ManipulationAtom


_SUPPORT_METADATA_KEY = "magnetic_support_object_ids"


def _annotate_candidate(
    candidate: PoseCandidate,
    support_ids: tuple[str, ...],
    atom: ManipulationAtom,
) -> PoseCandidate:
    return replace(
        candidate,
        metadata={
            **dict(candidate.metadata),
            _SUPPORT_METADATA_KEY: list(support_ids),
            "magnetic_connection_id": atom.metadata.get(
                "magnetic_connection_id"
            ),
            "magnetic_engagement_clearance_m": atom.metadata.get(
                "magnetic_engagement_clearance_m"
            ),
        },
    )


class MagneticPickPlaceTaskBuilder:
    """Annotate generic PickPlaceTask candidates with multi-support contact."""

    def __init__(self, base: FixedSceneAtomTaskBuilder) -> None:
        self.base = base

    def __call__(
        self,
        atom: ManipulationAtom,
        scene: TaskSceneState,
    ) -> PickPlaceTask:
        task = self.base(atom, scene)
        support_ids = tuple(
            dict.fromkeys(
                str(item)
                for item in atom.metadata.get(
                    _SUPPORT_METADATA_KEY,
                    (),
                )
                if item
            )
        )
        if not support_ids:
            return task
        missing = set(support_ids) - set(scene.objects)
        if missing:
            raise KeyError(
                f"magnetic support objects are missing from scene: {sorted(missing)}"
            )
        annotated_by_id = {
            item.candidate_id: _annotate_candidate(
                item,
                support_ids,
                atom,
            )
            for item in task.place_candidates
        }
        places_by_grasp = {
            grasp_id: tuple(
                annotated_by_id.get(
                    item.candidate_id,
                    _annotate_candidate(item, support_ids, atom),
                )
                for item in candidates
            )
            for grasp_id, candidates in task.place_candidates_by_grasp.items()
        }
        return replace(
            task,
            place_candidates=tuple(annotated_by_id.values()),
            place_candidates_by_grasp=places_by_grasp,
        )


def _support_ids(candidates: Iterable[PoseCandidate]) -> tuple[str, ...]:
    return tuple(
        dict.fromkeys(
            str(item)
            for candidate in candidates
            for item in candidate.metadata.get(_SUPPORT_METADATA_KEY, ())
            if item
        )
    )


class MagneticContactPlanningBackend:
    """Delegate planner calls while applying scoped multi-support contact rules."""

    def __init__(self, backend: Any) -> None:
        self.backend = backend

    def __getattr__(self, name: str) -> Any:
        return getattr(self.backend, name)

    def prepare_pose_candidates_coarse(
        self,
        candidates: tuple[PoseCandidate, ...],
        scene: PlanningScene,
        *,
        tool_frame: str = "gripper_tcp",
        ignore_object_name: str | None = None,
        ignore_object_names: tuple[str, ...] = (),
    ) -> None:
        support_ids = _support_ids(candidates)
        merged = tuple(
            dict.fromkeys(
                item
                for item in (
                    ignore_object_name,
                    *ignore_object_names,
                    *support_ids,
                )
                if item
            )
        )
        return self.backend.prepare_pose_candidates_coarse(
            candidates,
            scene,
            tool_frame=tool_frame,
            ignore_object_name=None,
            ignore_object_names=merged,
        )

    def prepare_pose_candidates(
        self,
        candidates: tuple[PoseCandidate, ...],
        scene: PlanningScene,
        *,
        tool_frame: str = "gripper_tcp",
        ignore_object_name: str | None = None,
        ignore_object_names: tuple[str, ...] = (),
    ) -> None:
        return self.prepare_pose_candidates_coarse(
            candidates,
            scene,
            tool_frame=tool_frame,
            ignore_object_name=ignore_object_name,
            ignore_object_names=ignore_object_names,
        )

    def resolve_pose_tolerance_candidates(
        self,
        candidates: tuple[PoseCandidate, ...],
        scene: PlanningScene,
        *,
        tool_frame: str = "gripper_tcp",
        ignore_object_names: tuple[str, ...] = (),
        disable_collision_links: tuple[str, ...] = (),
    ) -> tuple[PoseCandidate, ...]:
        support_ids = _support_ids(candidates)
        merged = tuple(
            dict.fromkeys((*ignore_object_names, *support_ids))
        )
        return self.backend.resolve_pose_tolerance_candidates(
            candidates,
            scene,
            tool_frame=tool_frame,
            ignore_object_names=merged,
            disable_collision_links=disable_collision_links,
        )

    def _filtered_contact_scene(
        self,
        scene: PlanningScene,
        support_ids: tuple[str, ...],
    ) -> PlanningScene:
        support_set = set(support_ids)
        return PlanningScene(
            tuple(
                item for item in scene.objects if item.name not in support_set
            ),
            revision=(
                None
                if scene.revision is None
                else f"{scene.revision}:magnetic-contact:"
                + ",".join(sorted(support_set))
            ),
        )

    def plan_linear_candidates(
        self,
        request: BatchPlanningRequest,
        *,
        axis: str = "z",
        project_distance_to_goal: bool = False,
        ignore_object_name: str | None = None,
        disable_collision_links: tuple[str, ...] | None = None,
        allow_start_contact_escape: bool = False,
    ) -> Any:
        support_ids = _support_ids(request.candidates)
        if not support_ids:
            return self.backend.plan_linear_candidates(
                request,
                axis=axis,
                project_distance_to_goal=project_distance_to_goal,
                ignore_object_name=ignore_object_name,
                disable_collision_links=disable_collision_links,
                allow_start_contact_escape=allow_start_contact_escape,
            )
        # Prefer the backend's scoped obstacle toggles when available.  The
        # filtered-scene fallback is limited to this one contact segment.
        update_scene = getattr(self.backend, "update_scene", None)
        obstacle_enabled = getattr(self.backend, "_obstacle_enabled", None)
        set_obstacle_enabled = getattr(
            self.backend,
            "_set_obstacle_enabled",
            None,
        )
        if (
            callable(update_scene)
            and callable(obstacle_enabled)
            and callable(set_obstacle_enabled)
        ):
            if getattr(self.backend, "_scene", None) is not request.scene:
                update_scene(request.scene)
            snapshot = {
                name: bool(obstacle_enabled(name)) for name in support_ids
            }
            try:
                for name, enabled in snapshot.items():
                    if enabled:
                        set_obstacle_enabled(name, False)
                return self.backend.plan_linear_candidates(
                    request,
                    axis=axis,
                    project_distance_to_goal=project_distance_to_goal,
                    ignore_object_name=None,
                    disable_collision_links=disable_collision_links,
                    allow_start_contact_escape=allow_start_contact_escape,
                )
            finally:
                for name, enabled in snapshot.items():
                    set_obstacle_enabled(name, enabled)
        filtered = self._filtered_contact_scene(
            request.scene,
            support_ids,
        )
        return self.backend.plan_linear_candidates(
            replace(request, scene=filtered),
            axis=axis,
            project_distance_to_goal=project_distance_to_goal,
            ignore_object_name=None,
            disable_collision_links=disable_collision_links,
            allow_start_contact_escape=allow_start_contact_escape,
        )
