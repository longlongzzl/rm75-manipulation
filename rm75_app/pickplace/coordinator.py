"""Planner-independent pick-place state machine.

The coordinator owns sequencing; perception, cuRobo and simulation remain
replaceable adapters.  In particular, attachment is an explicit planning
boundary and can no longer be hidden in a monolithic runtime script.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import time
from typing import TYPE_CHECKING, Any, Mapping, Protocol

import numpy as np

from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz
from rm75_app.planning.contracts import (
    BatchPlanningRequest,
    CandidatePlan,
    JointConfiguration,
    JointTrajectory,
    PlanningScene,
    Pose,
    PoseCandidate,
)
from rm75_app.planning.interfaces import PlanningBackend


# Contact spheres overlap thin objects at physical contact. Suspend them only
# while resolving grasp/place endpoints; every path keeps full collision.
_CONTACT_ENDPOINT_COLLISION_LINKS = (
    "left_pad",
    "right_pad",
    "gripper_Left_Support_Link",
    "gripper_Right_Support_Link",
)


if TYPE_CHECKING:
    from rm75_app.perception.held_object_refinement import (
        HeldObjectRefinementHook,
        HeldObjectRefinementUpdate,
    )
else:
    HeldObjectRefinementHook = Any
    HeldObjectRefinementUpdate = Any


class TrajectoryExecutor(Protocol):
    def execute_trajectory(self, stage: str, trajectory: JointTrajectory) -> None: ...

    def set_gripper(self, closed: bool) -> None: ...


def _cached_configuration(
    planner: Any,
    candidate: PoseCandidate,
    reference: JointConfiguration,
) -> JointConfiguration | None:
    """Read a backend IK cache while preserving older adapter compatibility."""

    resolver = getattr(planner, "configuration_for_pose_candidate", None)
    if not callable(resolver):
        return None
    try:
        return resolver(candidate, tuple(reference.names), reference)
    except TypeError:
        return resolver(candidate, tuple(reference.names))


def _cached_joint_distance(
    planner: Any,
    candidate: PoseCandidate,
    reference: JointConfiguration,
) -> float:
    """Return cached IK distance, or infinity when a backend has no IK cache."""

    configuration = _cached_configuration(planner, candidate, reference)
    if configuration is None:
        return float("inf")
    values = np.asarray(configuration.positions, dtype=np.float64)
    current = np.asarray(reference.positions, dtype=np.float64)
    if values.shape != current.shape:
        return float("inf")
    return float(np.linalg.norm(values - current))


def _trajectory_start_gap(
    trajectory: JointTrajectory,
    expected: JointConfiguration,
) -> float:
    if tuple(trajectory.joint_names) != tuple(expected.names):
        return float("inf")
    return float(
        np.max(
            np.abs(
                np.asarray(trajectory.positions[0], dtype=np.float64)
                - np.asarray(expected.positions, dtype=np.float64)
            )
        )
    )


@dataclass(frozen=True)
class PickPlaceTask:
    object_name: str
    current: JointConfiguration
    grasp_candidates: tuple[PoseCandidate, ...]
    place_candidates: tuple[PoseCandidate, ...]
    scene: PlanningScene
    placement_mode: str = "surface_place"
    tool_frame: str = "gripper_tcp"
    grasp_approach_offset: float = -0.10
    lift_height: float = 0.10
    place_clearance: float = 0.10
    max_attempts: int = 2
    place_contact_object_name: str | None = None
    max_motion_candidates: int = 8
    pregrasp_fallback_configuration: JointConfiguration | None = None
    place_candidates_by_grasp: Mapping[str, tuple[PoseCandidate, ...]] = field(default_factory=dict)
    enable_axis_fallback: bool = True
    candidate_build_time_s: float | None = None

    def __post_init__(self) -> None:
        if not self.object_name:
            raise ValueError("object_name must not be empty")
        if self.object_name not in {item.name for item in self.scene.objects}:
            raise ValueError(f"picked object {self.object_name!r} is missing from the scene")
        if not self.grasp_candidates or not self.place_candidates:
            raise ValueError("grasp and place candidates must not be empty")
        grasp_ids = {item.candidate_id for item in self.grasp_candidates}
        unknown = set(self.place_candidates_by_grasp) - grasp_ids
        if unknown:
            raise ValueError(f"place mapping contains unknown grasp ids: {sorted(unknown)}")
        if any(not candidates for candidates in self.place_candidates_by_grasp.values()):
            raise ValueError("per-grasp place candidate lists must not be empty")
        if self.max_motion_candidates < 1:
            raise ValueError("max_motion_candidates must be positive")
        object.__setattr__(
            self,
            "place_candidates_by_grasp",
            {str(key): tuple(value) for key, value in self.place_candidates_by_grasp.items()},
        )

    def places_for_grasp(self, grasp_id: str) -> tuple[PoseCandidate, ...]:
        return tuple(self.place_candidates_by_grasp.get(grasp_id, self.place_candidates))


@dataclass(frozen=True)
class ExecutedStage:
    name: str
    candidate_id: str | None
    end: JointConfiguration


@dataclass(frozen=True)
class PickPlaceRunResult:
    success: bool
    stages: tuple[ExecutedStage, ...]
    selected_grasp: str | None = None
    selected_place: str | None = None
    failure_stage: str | None = None
    message: str | None = None
    held_object_refinement: HeldObjectRefinementUpdate | None = None
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


def _offset_candidates(
    candidates: tuple[PoseCandidate, ...], dz: float, prefix: str
) -> tuple[PoseCandidate, ...]:
    return tuple(
        PoseCandidate(
            candidate_id=f"{prefix}:{candidate.candidate_id}",
            pose=Pose(
                [
                    candidate.pose.position[0],
                    candidate.pose.position[1],
                    candidate.pose.position[2] + dz,
                ],
                candidate.pose.quaternion_wxyz,
            ),
            score=candidate.score,
            metadata=candidate.metadata,
        )
        for candidate in candidates
    )


def _approach_offset_candidates(
    candidates: tuple[PoseCandidate, ...], distance: float, prefix: str = "pregrasp"
) -> tuple[PoseCandidate, ...]:
    """Move away from contact along each candidate's local -Z approach line."""

    output = []
    for candidate in candidates:
        w, x, y, z = np.asarray(candidate.pose.quaternion_wxyz, dtype=np.float64)
        rotation = np.asarray(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        approach = rotation[:, 2]
        position = np.asarray(candidate.pose.position, dtype=np.float64) - float(distance) * approach
        output.append(
            PoseCandidate(
                candidate_id=f"{prefix}:{candidate.candidate_id}",
                pose=Pose(position, candidate.pose.quaternion_wxyz),
                score=candidate.score,
                metadata={**dict(candidate.metadata), "offset_frame": "tool_minus_z"},
            )
        )
    return tuple(output)


def _place_approach_candidates(
    candidate: PoseCandidate, distance: float
) -> tuple[PoseCandidate, ...]:
    """Generate generic world/tool approaches at progressively shorter clearances."""
    minimum = max(
        0.0,
        float(candidate.metadata.get("required_retreat_clearance_m", 0.0)),
    )
    nominal = max(abs(float(distance)), minimum)
    distances: list[float] = []
    for value in (
        nominal,
        max(0.03, 0.75 * nominal),
        max(0.03, 0.50 * nominal),
        max(0.03, 0.30 * nominal),
    ):
        value = max(float(value), minimum)
        if not any(abs(value - existing) < 1.0e-6 for existing in distances):
            distances.append(value)
    output: list[PoseCandidate] = []
    for index, clearance in enumerate(distances):
        suffix = "" if index == 0 else f"_{clearance * 1000:.0f}mm"
        world_up = _offset_candidates(
            (candidate,), clearance, f"preplace_world_z{suffix}"
        )[0]
        tool_backoff = _approach_offset_candidates(
            (candidate,), clearance, f"preplace_tool_z{suffix}"
        )[0]
        metadata = {
            "preplace_clearance_m": clearance,
            "preplace_clearance_rank": index,
        }
        output.extend((
            replace(world_up, metadata={**dict(world_up.metadata), **metadata}),
            replace(tool_backoff, metadata={**dict(tool_backoff.metadata), **metadata}),
        ))
    return tuple(output)


def _pose_matrix(pose: Pose) -> np.ndarray:
    w, x, y, z = np.asarray(pose.quaternion_wxyz, dtype=np.float64)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    transform[:3, 3] = np.asarray(pose.position, dtype=np.float64)
    return transform


def _rebuild_places_for_resolved_grasp(
    original_grasp: PoseCandidate,
    resolved_grasp: PoseCandidate,
    places: tuple[PoseCandidate, ...],
) -> tuple[PoseCandidate, ...]:
    """Preserve the physical held-object transform after free-axis IK."""

    if not places:
        return ()
    original_relation = places[0].metadata.get("T_tcp_object")
    if original_relation is None:
        return places
    T_source_object = _pose_matrix(original_grasp.pose) @ np.asarray(
        original_relation, dtype=np.float64
    ).reshape(4, 4)
    T_resolved_tcp = _pose_matrix(resolved_grasp.pose)
    T_resolved_tcp_object = np.linalg.inv(T_resolved_tcp) @ T_source_object
    output: list[PoseCandidate] = []
    source_grasp_id = str(
        resolved_grasp.metadata.get(
            "source_grasp_candidate_id", resolved_grasp.candidate_id
        )
    )
    relation_suffix = (
        ""
        if resolved_grasp.candidate_id == source_grasp_id
        else resolved_grasp.candidate_id[len(source_grasp_id):]
    )
    for place in places:
        target = place.metadata.get(
            "planning_target_object_pose", place.metadata.get("target_object_pose")
        )
        if target is None:
            output.append(place)
            continue
        T_target_object = np.asarray(target, dtype=np.float64).reshape(4, 4)
        T_target_tcp = T_target_object @ np.linalg.inv(T_resolved_tcp_object)
        output.append(
            PoseCandidate(
                f"{place.candidate_id}{relation_suffix}",
                Pose(
                    T_target_tcp[:3, 3],
                    matrix_to_quaternion_wxyz(T_target_tcp[:3, :3]),
                ),
                score=place.score,
                metadata={
                    **dict(place.metadata),
                    "paired_grasp_id": resolved_grasp.candidate_id,
                    "source_place_candidate_id": place.candidate_id,
                    "T_tcp_object": T_resolved_tcp_object.tolist(),
                    "axis_constrained_grasp": True,
                    "resolved_grasp_pose": resolved_grasp.pose.as_curobo_list(),
                },
            )
        )
    return tuple(output)


def _target_object_pose(place_candidate: PoseCandidate) -> Pose:
    value = place_candidate.metadata.get(
        "planning_target_object_pose",
        place_candidate.metadata.get("target_object_pose"),
    )
    if value is None:
        # Compatibility for externally constructed tasks that predate explicit
        # TCP/object pose separation.
        return place_candidate.pose
    transform = np.asarray(value, dtype=np.float64).reshape(4, 4)
    return Pose(
        transform[:3, 3],
        matrix_to_quaternion_wxyz(transform[:3, :3]),
    )


def _reverse_trajectory(trajectory: JointTrajectory) -> JointTrajectory:
    """Return the exact time-reversed path for a verified contact retreat."""

    dt = trajectory.dt
    if isinstance(dt, np.ndarray):
        dt = np.asarray(dt)[::-1].copy()
    return JointTrajectory(
        trajectory.joint_names,
        trajectory.positions[::-1].copy(),
        dt=dt,
    )


class PickPlaceCoordinator:
    def __init__(
        self,
        planner: PlanningBackend,
        executor: TrajectoryExecutor,
        held_object_refiner: HeldObjectRefinementHook | None = None,
    ):
        self.planner = planner
        self.executor = executor
        self.held_object_refiner = held_object_refiner
        self._last_plan_failure: dict[str, Any] = {}

    @staticmethod
    def _with_axis_fallback_diagnostics(
        result: PickPlaceRunResult,
        primary: PickPlaceRunResult,
        *,
        input_count: int,
        resolved_count: int,
    ) -> PickPlaceRunResult:
        return replace(
            result,
            diagnostics={
                **dict(result.diagnostics),
                "planning_strategy": "discrete_then_axis_fallback",
                "discrete_primary": {
                    "failure_stage": primary.failure_stage,
                    "message": primary.message,
                    "diagnostics": dict(primary.diagnostics),
                },
                "axis_fallback": {
                    "attempted": True,
                    "input_count": int(input_count),
                    "resolved_count": int(resolved_count),
                },
            },
        )

    def _run_axis_fallback(
        self,
        task: PickPlaceTask,
        primary: PickPlaceRunResult,
        axis_resolver: Any,
    ) -> PickPlaceRunResult:
        """Retry a failed discrete chain with continuous-axis FK solutions."""

        original_grasps = tuple(task.grasp_candidates)
        original_by_id = {
            item.candidate_id: item for item in original_grasps
        }
        resolved_grasps = tuple(
            axis_resolver(
                original_grasps,
                task.scene,
                tool_frame=task.tool_frame,
                ignore_object_names=(task.object_name,),
                disable_collision_links=_CONTACT_ENDPOINT_COLLISION_LINKS,
            )
        )
        if not resolved_grasps:
            return self._with_axis_fallback_diagnostics(
                primary,
                primary,
                input_count=len(original_grasps),
                resolved_count=0,
            )

        places_by_grasp: dict[str, tuple[PoseCandidate, ...]] = {}
        deduplicated_places: dict[str, PoseCandidate] = {}
        for resolved in resolved_grasps:
            source_id = str(
                resolved.metadata.get(
                    "source_grasp_candidate_id", resolved.candidate_id
                )
            )
            source = original_by_id.get(source_id)
            if source is None:
                continue
            rebuilt = _rebuild_places_for_resolved_grasp(
                source,
                resolved,
                task.places_for_grasp(source_id),
            )
            if not rebuilt:
                continue
            places_by_grasp[resolved.candidate_id] = rebuilt
            for place in rebuilt:
                deduplicated_places[place.candidate_id] = place

        eligible_grasps = tuple(
            item for item in resolved_grasps if item.candidate_id in places_by_grasp
        )
        if not eligible_grasps or not deduplicated_places:
            return self._with_axis_fallback_diagnostics(
                primary,
                primary,
                input_count=len(original_grasps),
                resolved_count=len(resolved_grasps),
            )

        fallback = self.run(
            replace(
                task,
                grasp_candidates=eligible_grasps,
                place_candidates=tuple(deduplicated_places.values()),
                place_candidates_by_grasp=places_by_grasp,
                enable_axis_fallback=False,
            )
        )
        return self._with_axis_fallback_diagnostics(
            fallback,
            primary,
            input_count=len(original_grasps),
            resolved_count=len(resolved_grasps),
        )

    @staticmethod
    def _end_configuration(trajectory: JointTrajectory) -> JointConfiguration:
        return JointConfiguration(trajectory.joint_names, trajectory.positions[-1])

    def _execute(self, stage: str, plan: CandidatePlan) -> ExecutedStage:
        trajectory = plan.trajectory
        if trajectory is None:
            raise RuntimeError(f"successful {stage} plan has no trajectory")
        self.executor.execute_trajectory(stage, trajectory)
        return ExecutedStage(stage, plan.candidate_id, self._end_configuration(trajectory))

    def _plan_pose_stage(
        self,
        *,
        stage: str,
        current: JointConfiguration,
        candidates: tuple[PoseCandidate, ...],
        task: PickPlaceTask,
        prefer_unbiased_ik: bool = True,
        prefer_direct_tcp_path: bool = False,
    ) -> CandidatePlan | None:
        batch_size = max(1, int(getattr(self.planner, "max_batch_size", len(candidates))))
        plans: list[CandidatePlan] = []
        for start in range(0, len(candidates), batch_size):
            chunk = tuple(candidates[start : start + batch_size])
            result = self.planner.plan_candidates(
                BatchPlanningRequest(
                    current=current,
                    candidates=chunk,
                    scene=task.scene,
                    tool_frame=task.tool_frame,
                    max_attempts=task.max_attempts,
                    prefer_unbiased_ik=prefer_unbiased_ik,
                    prefer_direct_tcp_path=prefer_direct_tcp_path,
                )
            )
            plans.extend(result.plans)
        score_by_id = {item.candidate_id: item.score for item in candidates}
        feasible = [item for item in plans if item.success]
        best = (
            None
            if not feasible
            else max(
                feasible,
                key=lambda item: score_by_id.get(item.candidate_id, float("-inf")),
            )
        )
        if best is None:
            diagnostics = ", ".join(
                f"{plan.candidate_id}={plan.status}" for plan in plans
            )
            print(f"[{stage}] no feasible candidate: {diagnostics}")
            self._last_plan_failure = {
                "stage": stage,
                "candidates": [
                    {
                        "candidate_id": plan.candidate_id,
                        "status": plan.status,
                        **dict(plan.diagnostics),
                    }
                    for plan in plans
                ],
            }
        else:
            self._last_plan_failure = {}
        return best

    def _plan_linear_stage(
        self,
        *,
        stage: str,
        current: JointConfiguration,
        candidate: PoseCandidate,
        task: PickPlaceTask,
        ignore_object_name: str | None = None,
        axis: str = "z",
        allow_start_contact_escape: bool = False,
        project_distance_to_goal: bool = False,
        disable_collision_links: tuple[str, ...] | None = None,
    ) -> CandidatePlan | None:
        request = BatchPlanningRequest(
            current=current,
            candidates=(candidate,),
            scene=task.scene,
            tool_frame=task.tool_frame,
            max_attempts=task.max_attempts,
        )
        linear = getattr(self.planner, "plan_linear_candidates", None)
        if callable(linear):
            result = linear(
                request,
                axis=axis,
                project_distance_to_goal=project_distance_to_goal,
                ignore_object_name=ignore_object_name,
                disable_collision_links=disable_collision_links,
                allow_start_contact_escape=allow_start_contact_escape,
            )
            best = result.best((candidate,))
        else:
            result = self.planner.plan_candidates(request)
            best = result.best((candidate,))
        if best is None:
            self._last_plan_failure = {
                "stage": stage,
                "candidates": [
                    {
                        "candidate_id": item.candidate_id,
                        "status": item.status,
                        **dict(item.diagnostics),
                    }
                    for item in result.plans
                ],
            }
        else:
            self._last_plan_failure = {}
        return best

    def _run_segmented_chain(
        self,
        task: PickPlaceTask,
        relation_grasp_candidates: tuple[PoseCandidate, ...],
        complete_places_by_grasp: Mapping[str, tuple[PoseCandidate, ...]],
        grasp_scores: Mapping[str, float],
        relation_screen: Mapping[str, Any],
    ) -> PickPlaceRunResult:
        """Run the proven segmented chain without cuRobo2 PlanGrasp.

        Free-space legs use normal MotionGen.  Contact-adjacent legs use the
        independent world-Z linear primitive, and broad reachability has
        already been screened by the fixed batch64 IK solver.
        """

        planning_started = time.perf_counter()
        pregrasps_by_grasp = {
            item.candidate_id: _approach_offset_candidates(
                (item,), abs(float(task.grasp_approach_offset))
            )[0]
            for item in relation_grasp_candidates
        }
        grasp_distances = {
            item.candidate_id: _cached_joint_distance(
                self.planner,
                pregrasps_by_grasp[item.candidate_id],
                task.current,
            )
            for item in relation_grasp_candidates
        }
        ranked_all = sorted(
            relation_grasp_candidates,
            key=lambda item: (
                np.isfinite(grasp_distances[item.candidate_id]),
                -grasp_distances[item.candidate_id],
                grasp_scores.get(item.candidate_id, item.score),
            ),
            reverse=True,
        )
        primary: list[PoseCandidate] = []
        secondary: list[PoseCandidate] = []
        seen_sources: set[str] = set()
        for item in ranked_all:
            source_id = str(
                item.metadata.get("source_grasp_candidate_id", item.candidate_id)
            )
            if source_id in seen_sources:
                secondary.append(item)
            else:
                seen_sources.add(source_id)
                primary.append(item)
        ranked_grasps = (primary + secondary)[: task.max_motion_candidates]
        selected_chain = None
        failures: list[dict[str, Any]] = []
        attached = False
        fallback_home_plan: CandidatePlan | None = None
        fallback_home_attempted = False
        for grasp_candidate in ranked_grasps:
            pregrasp_candidate = pregrasps_by_grasp[grasp_candidate.candidate_id]
            pregrasp = self._plan_pose_stage(
                stage="pregrasp",
                current=task.current,
                candidates=(pregrasp_candidate,),
                task=task,
            )
            pregrasp_entry: CandidatePlan | None = None
            if pregrasp is None or pregrasp.trajectory is None:
                direct_failure = dict(self._last_plan_failure)
                fallback = task.pregrasp_fallback_configuration
                plan_to_configuration = getattr(
                    self.planner, "plan_to_configuration", None
                )
                fallback_is_distinct = (
                    fallback is not None
                    and tuple(fallback.names) == tuple(task.current.names)
                    and not np.allclose(
                        fallback.positions, task.current.positions, atol=1.0e-6
                    )
                )
                if not callable(plan_to_configuration) or not fallback_is_distinct:
                    failures.append(direct_failure)
                    continue
                if not fallback_home_attempted:
                    fallback_home_attempted = True
                    fallback_home_plan = plan_to_configuration(
                        task.current,
                        fallback,
                        task.scene,
                        max_attempts=task.max_attempts,
                        candidate_id="pregrasp_fallback_initial",
                    )
                if (
                    fallback_home_plan is None
                    or not fallback_home_plan.success
                    or fallback_home_plan.trajectory is None
                ):
                    failures.extend(
                        (
                            direct_failure,
                            {
                                "stage": "pregrasp_fallback_initial",
                                "candidates": [
                                    {
                                        "candidate_id": "pregrasp_fallback_initial",
                                        "status": (
                                            "not_available"
                                            if fallback_home_plan is None
                                            else fallback_home_plan.status
                                        ),
                                    }
                                ],
                            },
                        )
                    )
                    continue
                pregrasp = self._plan_pose_stage(
                    stage="pregrasp_from_initial",
                    current=fallback,
                    candidates=(pregrasp_candidate,),
                    task=task,
                )
                if pregrasp is None or pregrasp.trajectory is None:
                    failures.extend((direct_failure, dict(self._last_plan_failure)))
                    continue
                pregrasp_entry = fallback_home_plan
            pregrasp_end = self._end_configuration(pregrasp.trajectory)
            grasp = self._plan_linear_stage(
                stage="grasp",
                current=pregrasp_end,
                candidate=grasp_candidate,
                task=task,
                ignore_object_name=task.object_name,
            )
            if grasp is None or grasp.trajectory is None:
                failures.append(dict(self._last_plan_failure))
                continue
            grasp_end = self._end_configuration(grasp.trajectory)
            self.planner.attach_object(task.object_name, grasp_end)
            attached = True
            try:
                resolve_tool_pose = getattr(
                    self.planner, "tool_pose_for_configuration", None
                )
                lift_origin = grasp_candidate
                if callable(resolve_tool_pose):
                    lift_origin = PoseCandidate(
                        grasp_candidate.candidate_id,
                        resolve_tool_pose(grasp_end, task.tool_frame),
                        score=grasp_candidate.score,
                        metadata={**grasp_candidate.metadata, "lift_origin": "actual_fk"},
                    )
                retreat_candidates = (
                    (
                        _offset_candidates(
                            (lift_origin,), abs(float(task.lift_height)), "lift_world_z"
                        )[0],
                        "z",
                        False,
                    ),
                    (
                        _approach_offset_candidates(
                            (lift_origin,), abs(float(task.lift_height)), "lift_tool_z"
                        )[0],
                        "z",
                        True,
                    ),
                )
                lift = None
                retreat_failures: list[dict[str, Any]] = []
                for retreat_candidate, retreat_axis, project_to_goal in retreat_candidates:
                    lift = self._plan_linear_stage(
                        stage="lift",
                        current=grasp_end,
                        candidate=retreat_candidate,
                        allow_start_contact_escape=True,
                        axis=retreat_axis,
                        project_distance_to_goal=project_to_goal,
                        task=task,
                    )
                    if lift is not None and lift.trajectory is not None:
                        break
                    retreat_failures.append(dict(self._last_plan_failure))
                if lift is None or lift.trajectory is None:
                    failures.extend(retreat_failures)
                    continue
                lift_end = self._end_configuration(lift.trajectory)
                preplace_options_by_place = {
                    item.candidate_id: _place_approach_candidates(
                        item, task.place_clearance
                    )
                    for item in complete_places_by_grasp[
                        grasp_candidate.candidate_id
                    ]
                }
                preplace_distances_by_place = {
                    item.candidate_id: {
                        option.candidate_id: _cached_joint_distance(
                            self.planner, option, lift_end
                        )
                        for option in preplace_options_by_place[item.candidate_id]
                    }
                    for item in complete_places_by_grasp[
                        grasp_candidate.candidate_id
                    ]
                }
                place_distances = {
                    candidate_id: min(distances.values(), default=float("inf"))
                    for candidate_id, distances in preplace_distances_by_place.items()
                }
                ranked_places = sorted(
                    complete_places_by_grasp[grasp_candidate.candidate_id],
                    key=lambda item: (
                        np.isfinite(place_distances[item.candidate_id]),
                        -place_distances[item.candidate_id],
                        item.score,
                    ),
                    reverse=True,
                )[: task.max_motion_candidates]
                for place_candidate in ranked_places:
                    preplace_options = sorted(
                        preplace_options_by_place[place_candidate.candidate_id],
                        key=lambda item: preplace_distances_by_place[
                            place_candidate.candidate_id
                        ][item.candidate_id],
                    )
                    for preplace_candidate in preplace_options:
                        preplace = self._plan_pose_stage(
                            stage="preplace",
                            current=lift_end,
                            candidates=(preplace_candidate,),
                            task=task,
                            prefer_direct_tcp_path=(
                                task.placement_mode == "surface_place"
                            ),
                        )
                        if preplace is None or preplace.trajectory is None:
                            failures.append(dict(self._last_plan_failure))
                            continue
                        preplace_end = self._end_configuration(preplace.trajectory)
                        tool_frame_approach = preplace_candidate.candidate_id.startswith(
                            "preplace_tool_z"
                        )
                        place = self._plan_linear_stage(
                            stage="place",
                            current=preplace_end,
                            candidate=place_candidate,
                            task=task,
                            axis="z",
                            project_distance_to_goal=tool_frame_approach,
                            ignore_object_name=task.place_contact_object_name,
                        )
                        if place is None or place.trajectory is None:
                            place_start = _cached_configuration(
                                self.planner,
                                place_candidate,
                                preplace_end,
                            )
                            reverse_place = None
                            if place_start is not None:
                                reverse_place = self._plan_linear_stage(
                                    stage="place_reverse_probe",
                                    current=place_start,
                                    candidate=preplace_candidate,
                                    task=task,
                                    axis="z",
                                    project_distance_to_goal=tool_frame_approach,
                                    ignore_object_name=task.place_contact_object_name,
                                    allow_start_contact_escape=True,
                                )
                            if (
                                reverse_place is not None
                                and reverse_place.trajectory is not None
                            ):
                                reversed_trajectory = _reverse_trajectory(
                                    reverse_place.trajectory
                                )
                                start_gap = _trajectory_start_gap(
                                    reversed_trajectory, preplace_end
                                )
                                if start_gap > 0.10:
                                    plan_to_configuration = getattr(
                                        self.planner,
                                        "plan_to_configuration",
                                        None,
                                    )
                                    if callable(plan_to_configuration):
                                        reverse_pre_configuration = JointConfiguration(
                                            tuple(reversed_trajectory.joint_names),
                                            np.asarray(
                                                reversed_trajectory.positions[0],
                                                dtype=np.float64,
                                            ),
                                        )
                                        matching_preplace = plan_to_configuration(
                                            lift_end,
                                            reverse_pre_configuration,
                                            task.scene,
                                            max_attempts=task.max_attempts,
                                            candidate_id=(
                                                "preplace_matching_reverse_branch:"
                                                f"{place_candidate.candidate_id}"
                                            ),
                                        )
                                        if (
                                            matching_preplace.success
                                            and matching_preplace.trajectory is not None
                                        ):
                                            matching_end = self._end_configuration(
                                                matching_preplace.trajectory
                                            )
                                            matching_gap = _trajectory_start_gap(
                                                reversed_trajectory, matching_end
                                            )
                                            if matching_gap <= 0.10:
                                                preplace = matching_preplace
                                                preplace_end = matching_end
                                                start_gap = matching_gap
                                if start_gap <= 0.10:
                                    place = CandidatePlan(
                                        place_candidate.candidate_id,
                                        True,
                                        trajectory=reversed_trajectory,
                                        status="reverse_validated_place_approach",
                                        diagnostics={
                                            "reverse_start_gap_rad": start_gap
                                        },
                                    )
                                else:
                                    failures.append(
                                        {
                                            "stage": "place_reverse_probe",
                                            "candidates": [
                                                {
                                                    "candidate_id": place_candidate.candidate_id,
                                                    "status": "trajectory_discontinuity",
                                                    "start_gap_rad": start_gap,
                                                }
                                            ],
                                        }
                                    )
                                    continue
                            else:
                                failures.append(dict(self._last_plan_failure))
                                continue
                        selected_chain = (
                            grasp_candidate,
                            place_candidate,
                            pregrasp_entry,
                            pregrasp,
                            grasp,
                            lift,
                            preplace,
                            place,
                        )
                        break
                    if selected_chain is not None:
                        break
                if selected_chain is not None:
                    break
            finally:
                if attached:
                    self.planner.detach_object(task.object_name)
                    attached = False

        planning_time_s = time.perf_counter() - planning_started
        if selected_chain is None:
            return PickPlaceRunResult(
                False,
                (),
                failure_stage="segmented_chain",
                message="no complete segmented pick-place chain",
                diagnostics={
                    "relation_screen": dict(relation_screen),
                    "candidate_failures": failures,
                    "timing": {"segmented_plan_time_s": planning_time_s},
                },
            )

        (
            grasp_candidate,
            place_candidate,
            pregrasp_entry,
            pregrasp,
            grasp,
            lift,
            preplace,
            place,
        ) = selected_chain
        stages: list[ExecutedStage] = []
        released_collision_disabled = False
        held_refinement = None
        try:
            if pregrasp_entry is not None:
                stages.append(
                    self._execute("pregrasp_fallback_initial", pregrasp_entry)
                )
            stage = self._execute("approach", pregrasp)
            stages.append(stage)
            stage = self._execute("grasp", grasp)
            stages.append(stage)
            current = stage.end
            self.executor.set_gripper(True)
            self.planner.attach_object(task.object_name, current)
            attached = True

            stage = self._execute("lift", lift)
            stages.append(stage)
            current = stage.end
            if self.held_object_refiner is not None:
                try:
                    held_refinement = self.held_object_refiner.refine_after_lift(
                        task.object_name, current
                    )
                    if (
                        held_refinement.accepted
                        and held_refinement.T_tcp_object is not None
                    ):
                        self.planner.update_attached_object_pose(
                            task.object_name,
                            current,
                            held_refinement.T_tcp_object,
                        )
                except Exception as exc:
                    from rm75_app.perception.held_object_refinement import (
                        HeldObjectRefinementUpdate as _Update,
                    )

                    held_refinement = _Update(
                        False,
                        task.object_name,
                        None,
                        0.0,
                        0.0,
                        f"refinement_error:{type(exc).__name__}",
                        metadata={"error": str(exc)},
                    )

            stage = self._execute("preplace", preplace)
            stages.append(stage)
            stage = self._execute("place", place)
            stages.append(stage)
            self.executor.set_gripper(False)
            released_pose = _target_object_pose(place_candidate)
            self.planner.detach_object(task.object_name, released_pose)
            attached = False
            released_collision_disabled = True

            retreat = CandidatePlan(
                f"retreat:{place_candidate.candidate_id}",
                True,
                trajectory=_reverse_trajectory(place.trajectory),
                status="reverse_validated_place_line",
            )
            stages.append(self._execute("retreat", retreat))
            self.planner.enable_object_collision(task.object_name)
            released_collision_disabled = False
            return PickPlaceRunResult(
                True,
                tuple(stages),
                selected_grasp=grasp_candidate.candidate_id,
                selected_place=place_candidate.candidate_id,
                held_object_refinement=held_refinement,
                diagnostics={
                    "relation_screen": dict(relation_screen),
                    "planner_mode": "batch64_ik_segmented_motiongen",
                    "pregrasp_initial_fallback_used": pregrasp_entry is not None,
                    "timing": {"segmented_plan_time_s": planning_time_s},
                },
            )
        finally:
            if attached:
                self.planner.detach_object(task.object_name)
            elif released_collision_disabled:
                self.planner.enable_object_collision(task.object_name)

    def run(self, task: PickPlaceTask) -> PickPlaceRunResult:
        coarse_screening_active = False
        set_gripper_collision_state = getattr(
            self.planner, "set_gripper_collision_state", None
        )
        try:
            if callable(set_gripper_collision_state):
                set_gripper_collision_state(False)
            begin_coarse = getattr(self.planner, "begin_coarse_screening", None)
            end_coarse = getattr(self.planner, "end_coarse_screening", None)
            if callable(begin_coarse):
                begin_coarse()
                coarse_screening_active = callable(end_coarse)
            prepare = getattr(self.planner, "prepare_pose_candidates", None)
            prepare_coarse = getattr(
                self.planner, "prepare_pose_candidates_coarse", None
            ) or prepare
            coarse_ik_batch_size = int(
                getattr(
                    getattr(self.planner, "config", None),
                    "coarse_ik_batch_size",
                    0,
                )
                or 0
            )
            coarse_ik_metrics = {
                "coarse_ik_call_count": 0,
                "coarse_ik_rows_requested": 0,
                "coarse_ik_rows_padded": 0,
                "stable_ik_call_count": 0,
            }

            def prepare_coarse_endpoints(
                candidates: tuple[PoseCandidate, ...],
                *,
                ignore_object_names: tuple[str, ...],
            ) -> None:
                """Screen one same-context endpoint family and record its batch cost."""

                if not callable(prepare_coarse) or not candidates:
                    return
                requested_rows = len(candidates)
                if coarse_ik_batch_size > 0:
                    solver_calls = (
                        requested_rows + coarse_ik_batch_size - 1
                    ) // coarse_ik_batch_size
                    padded_rows = solver_calls * coarse_ik_batch_size
                else:
                    # Generic planners do not expose fixed-shape padding.
                    solver_calls = 1
                    padded_rows = requested_rows
                coarse_ik_metrics["coarse_ik_call_count"] += solver_calls
                coarse_ik_metrics["coarse_ik_rows_requested"] += requested_rows
                coarse_ik_metrics["coarse_ik_rows_padded"] += padded_rows
                prepare_coarse(
                    candidates,
                    task.scene,
                    tool_frame=task.tool_frame,
                    ignore_object_names=ignore_object_names,
                )

            feasible_pose_ids = getattr(
                self.planner, "feasible_pose_candidate_ids", None
            )
            contact_ignores = tuple(
                name
                for name in (task.object_name, task.place_contact_object_name)
                if name
            )
            grasp_ignores = (task.object_name,)

            # A place TCP is only meaningful for the grasp transform that
            # produced it. Screen every grasp relation first, then expand and
            # evaluate place poses only for viable held-object transforms.
            grasp_screen_started = time.perf_counter()
            original_grasp_candidates = tuple(task.grasp_candidates)
            all_grasp_candidates = original_grasp_candidates
            runtime_places_by_grasp = {
                item.candidate_id: task.places_for_grasp(item.candidate_id)
                for item in original_grasp_candidates
            }
            axis_resolver = getattr(
                self.planner, "resolve_axis_constrained_pose_candidates", None
            )
            axis_requested = bool(
                task.enable_axis_fallback
                and callable(axis_resolver)
                and any(
                    item.metadata.get("free_rotation_axis_local")
                    for item in original_grasp_candidates
                )
            )
            axis_resolution = {
                "enabled": axis_requested,
                "strategy": "discrete_primary",
                "attempted": False,
                "input_count": len(original_grasp_candidates),
                "resolved_count": len(original_grasp_candidates),
            }

            def places_for_grasp(grasp_id: str) -> tuple[PoseCandidate, ...]:
                return tuple(runtime_places_by_grasp.get(grasp_id, ()))

            all_pregrasp_candidates = _approach_offset_candidates(
                all_grasp_candidates, abs(float(task.grasp_approach_offset))
            )
            prepare_coarse_endpoints(
                all_pregrasp_candidates + all_grasp_candidates,
                ignore_object_names=grasp_ignores,
            )
            if callable(feasible_pose_ids):
                pregrasp_feasible = feasible_pose_ids(all_pregrasp_candidates)
                grasp_feasible = feasible_pose_ids(all_grasp_candidates)
            else:
                pregrasp_feasible = frozenset(
                    item.candidate_id for item in all_pregrasp_candidates
                )
                grasp_feasible = frozenset(
                    item.candidate_id for item in all_grasp_candidates
                )
            raw_grasp_feasible = frozenset(grasp_feasible)
            raw_pregrasp_feasible = frozenset(pregrasp_feasible)
            grasp_ready_candidates = tuple(
                item
                for item in all_grasp_candidates
                if item.candidate_id in grasp_feasible
                and f"pregrasp:{item.candidate_id}" in pregrasp_feasible
            )
            grasp_ready_ids = {
                item.candidate_id for item in grasp_ready_candidates
            }
            grasp_screen_time_s = time.perf_counter() - grasp_screen_started
            metrics_reader = getattr(self.planner, "pose_candidate_metrics", None)

            def endpoint_summary(candidates: tuple[PoseCandidate, ...]) -> dict[str, Any]:
                metrics = metrics_reader(candidates) if callable(metrics_reader) else {}
                counts = {
                    "success": 0,
                    "constraint_rejected": 0,
                    "pose_not_converged": 0,
                    "missing_metrics": 0,
                }
                failed: list[dict[str, Any]] = []
                for candidate in candidates:
                    item = metrics.get(candidate.candidate_id)
                    if item is None:
                        counts["missing_metrics"] += 1
                        continue
                    if bool(item.get("success", False)):
                        counts["success"] += 1
                        continue
                    reason = (
                        "pose_not_converged"
                        if bool(item.get("constraint_feasible", False))
                        else "constraint_rejected"
                    )
                    counts[reason] += 1
                    failed.append(
                        {
                            "candidate_id": candidate.candidate_id,
                            "reason": reason,
                            "position_error_m": float(item.get("position_error_m", 0.0)),
                            "orientation_error_rad": float(item.get("orientation_error_rad", 0.0)),
                            "normalized_pose_gap": float(item.get("normalized_pose_gap", 0.0)),
                        }
                    )
                failed.sort(key=lambda item: item["normalized_pose_gap"])
                return {"reason_counts": counts, "nearest_failures": failed[:16]}

            grasp_endpoint_summary = endpoint_summary(all_grasp_candidates)
            pregrasp_endpoint_summary = endpoint_summary(all_pregrasp_candidates)
            collision_reader = getattr(
                self.planner, "diagnose_pose_candidate_collisions", None
            )
            collision_diagnostics: dict[str, Any] = {}
            if not grasp_ready_candidates and callable(collision_reader):
                collision_started = time.perf_counter()
                collision_diagnostics = {
                    "grasp": collision_reader(
                        all_grasp_candidates,
                        ignored_world_objects=grasp_ignores,
                    ),
                    "pregrasp": collision_reader(
                        all_pregrasp_candidates,
                        ignored_world_objects=grasp_ignores,
                    ),
                }
                collision_diagnostics["diagnostic_time_s"] = (
                    time.perf_counter() - collision_started
                )

            declared_place_candidates = tuple(task.place_candidates)
            if task.place_candidates_by_grasp:
                deduplicated: dict[str, PoseCandidate] = {
                    item.candidate_id: item for item in declared_place_candidates
                }
                for candidates in task.place_candidates_by_grasp.values():
                    for item in candidates:
                        deduplicated[item.candidate_id] = item
                declared_place_candidates = tuple(deduplicated.values())
                eligible: dict[str, PoseCandidate] = {}
                for grasp_id in grasp_ready_ids:
                    for item in places_for_grasp(grasp_id):
                        eligible[item.candidate_id] = item
                all_place_candidates = tuple(eligible.values())
            else:
                all_place_candidates = declared_place_candidates
            place_manifold_resolver = getattr(
                self.planner, "resolve_pose_tolerance_candidates", None
            )
            place_manifold_requested = any(
                item.metadata.get("continuous_place_manifold")
                for item in all_place_candidates
            )
            place_manifold_resolution = {
                "enabled": bool(
                    callable(place_manifold_resolver) and place_manifold_requested
                ),
                "input_count": len(all_place_candidates),
                "resolved_count": len(all_place_candidates),
            }
            place_manifold_input_candidates = all_place_candidates
            if (
                callable(place_manifold_resolver)
                and place_manifold_requested
                and all_place_candidates
            ):
                resolved_places = place_manifold_resolver(
                    all_place_candidates,
                    task.scene,
                    tool_frame=task.tool_frame,
                    ignore_object_names=contact_ignores,
                    disable_collision_links=_CONTACT_ENDPOINT_COLLISION_LINKS,
                )
                resolved_by_id = {
                    item.candidate_id: item for item in resolved_places
                }
                runtime_places_by_grasp = {
                    grasp_id: tuple(
                        resolved_by_id[item.candidate_id]
                        for item in candidates
                        if item.candidate_id in resolved_by_id
                    )
                    for grasp_id, candidates in runtime_places_by_grasp.items()
                }
                all_place_candidates = tuple(resolved_places)
                place_manifold_resolution["resolved_count"] = len(resolved_places)
                place_manifold_resolution[
                    "contact_endpoint_collision_links_ignored"
                ] = list(_CONTACT_ENDPOINT_COLLISION_LINKS)
                manifold_diagnostics = getattr(
                    self.planner, "pose_tolerance_resolution_diagnostics", None
                )
                if callable(manifold_diagnostics):
                    place_manifold_resolution["solver_diagnostics"] = (
                        manifold_diagnostics()
                    )
            preplace_by_place_id = {
                place.candidate_id: _place_approach_candidates(
                    place, task.place_clearance
                )
                for place in all_place_candidates
            }
            all_preplace_candidates = tuple(
                item for options in preplace_by_place_id.values() for item in options
            )
            max_tier = max(
                [int(item.metadata.get("search_tier", 0)) for item in task.grasp_candidates]
                + [int(item.metadata.get("search_tier", 0)) for item in all_place_candidates]
                + [0]
            )
            tiers = range(max_tier + 1) if callable(prepare) else (max_tier,)
            active_refinement_parent_ids: set[str] | None = {
                str(parent)
                for item in grasp_ready_candidates
                if (parent := item.metadata.get("refinement_parent_id")) is not None
            } or None
            available_refinement_parent_ids = {
                str(parent)
                for item in task.grasp_candidates
                if (parent := item.metadata.get("refinement_parent_id")) is not None
            }
            refinement_tier_by_parent: dict[str, int] = {}
            for item in task.grasp_candidates:
                parent = item.metadata.get("refinement_parent_id")
                if parent is None:
                    continue
                parent_id = str(parent)
                refinement_tier_by_parent[parent_id] = max(
                    refinement_tier_by_parent.get(parent_id, 0),
                    int(item.metadata.get("search_tier", 0)),
                )

            def tier_enabled(candidate: PoseCandidate, tier: int) -> bool:
                if int(candidate.metadata.get("search_tier", 0)) > tier:
                    return False
                parent = candidate.metadata.get("refinement_parent_id")
                return parent is None or (
                    active_refinement_parent_ids is not None
                    and str(parent) in active_refinement_parent_ids
                )
            screened_place_ids: set[str] = set()
            screened_grasp_ids: set[str] = {
                item.candidate_id for item in all_grasp_candidates
            }
            preplace_feasible: frozenset[str] = frozenset()
            place_feasible: frozenset[str] = frozenset()
            complete_places_by_grasp: dict[str, tuple[PoseCandidate, ...]] = {}
            relation_grasp_candidates: tuple[PoseCandidate, ...] = ()
            place_ready_grasps: tuple[PoseCandidate, ...] = ()
            selected_search_tier = max_tier
            place_screen_started = time.perf_counter()
            for tier in tiers:
                # The previous tier switches to the open gripper before
                # screening grasp/pregrasp endpoints. Restore the held-object
                # state before every place/preplace tier so their cached IK
                # keys remain valid after attach_object() closes the gripper.
                if callable(set_gripper_collision_state):
                    set_gripper_collision_state(True)
                cumulative_places = tuple(
                    item
                    for item in all_place_candidates
                    if tier_enabled(item, tier)
                )
                new_places = tuple(
                    item
                    for item in cumulative_places
                    if item.candidate_id not in screened_place_ids
                )
                new_preplaces = tuple(
                    preplace
                    for item in new_places
                    for preplace in preplace_by_place_id[item.candidate_id]
                )
                prepare_coarse_endpoints(
                    new_preplaces + new_places,
                    ignore_object_names=contact_ignores,
                )
                screened_place_ids.update(item.candidate_id for item in new_places)
                cumulative_preplaces = tuple(
                    preplace
                    for item in cumulative_places
                    for preplace in preplace_by_place_id[item.candidate_id]
                )
                if callable(feasible_pose_ids):
                    preplace_feasible = feasible_pose_ids(cumulative_preplaces)
                    place_feasible = feasible_pose_ids(cumulative_places)
                else:
                    preplace_feasible = frozenset(
                        item.candidate_id for item in cumulative_preplaces
                    )
                    place_feasible = frozenset(item.candidate_id for item in cumulative_places)

                place_ready_grasps = tuple(
                    grasp
                    for grasp in all_grasp_candidates
                    if tier_enabled(grasp, tier)
                    and any(
                        place.candidate_id in place_feasible
                        and any(
                            item.candidate_id in preplace_feasible
                            for item in preplace_by_place_id[place.candidate_id]
                        )
                        for place in places_for_grasp(grasp.candidate_id)
                    )
                )
                new_grasps = tuple(
                    item
                    for item in place_ready_grasps
                    if item.candidate_id not in screened_grasp_ids
                )
                if callable(set_gripper_collision_state):
                    set_gripper_collision_state(False)
                new_pregrasps = _approach_offset_candidates(
                    new_grasps, abs(float(task.grasp_approach_offset))
                )
                prepare_coarse_endpoints(
                    new_pregrasps + new_grasps,
                    ignore_object_names=grasp_ignores,
                )
                screened_grasp_ids.update(item.candidate_id for item in new_grasps)
                cumulative_pregrasps = _approach_offset_candidates(
                    place_ready_grasps,
                    abs(float(task.grasp_approach_offset)),
                )
                if callable(feasible_pose_ids):
                    pregrasp_feasible = feasible_pose_ids(cumulative_pregrasps)
                    grasp_feasible = feasible_pose_ids(place_ready_grasps)
                else:
                    pregrasp_feasible = frozenset(
                        item.candidate_id for item in cumulative_pregrasps
                    )
                    grasp_feasible = frozenset(
                        item.candidate_id for item in place_ready_grasps
                    )

                complete_places_by_grasp = {}
                for grasp_candidate in place_ready_grasps:
                    grasp_id = grasp_candidate.candidate_id
                    if (
                        grasp_id not in grasp_feasible
                        or f"pregrasp:{grasp_id}" not in pregrasp_feasible
                    ):
                        complete_places_by_grasp[grasp_id] = ()
                        continue
                    complete_places_by_grasp[grasp_id] = tuple(
                        place
                        for place in places_for_grasp(grasp_id)
                        if tier_enabled(place, tier)
                        and place.candidate_id in place_feasible
                        and any(
                            item.candidate_id in preplace_feasible
                            for item in preplace_by_place_id[place.candidate_id]
                        )
                    )
                relation_grasp_candidates = tuple(
                    candidate
                    for candidate in place_ready_grasps
                    if complete_places_by_grasp.get(candidate.candidate_id)
                )
                if relation_grasp_candidates:
                    boundary_relations = tuple(
                        candidate
                        for candidate in relation_grasp_candidates
                        if abs(float(candidate.metadata.get("axis_shift_m", 0.0)))
                        >= 0.035
                        and candidate.candidate_id in available_refinement_parent_ids
                        and refinement_tier_by_parent.get(candidate.candidate_id, 0)
                        > tier
                    )
                    if tier < max_tier and boundary_relations:
                        active_refinement_parent_ids = {
                            item.candidate_id
                            for item in sorted(
                                boundary_relations,
                                key=lambda item: item.score,
                                reverse=True,
                            )[:4]
                        }
                        continue
                    selected_search_tier = tier
                    break
                if tier < max_tier:
                    metrics_reader = getattr(
                        self.planner, "pose_candidate_metrics", None
                    )
                    if callable(metrics_reader) and place_ready_grasps:
                        grasp_metrics = metrics_reader(place_ready_grasps)
                        pregrasp_metrics = metrics_reader(cumulative_pregrasps)

                        def residual(candidate: PoseCandidate) -> float:
                            grasp_metric = grasp_metrics.get(candidate.candidate_id, {})
                            pregrasp_metric = pregrasp_metrics.get(
                                f"pregrasp:{candidate.candidate_id}", {}
                            )
                            total = 0.0
                            for metric in (grasp_metric, pregrasp_metric):
                                if not metric:
                                    total += 1.0e9
                                    continue
                                if not bool(metric.get("constraint_feasible", False)):
                                    total += 1.0e6
                                total += float(metric.get("normalized_pose_gap", 1.0e8))
                            return total

                        refinable = tuple(
                            item
                            for item in place_ready_grasps
                            if item.candidate_id in available_refinement_parent_ids
                            and refinement_tier_by_parent.get(item.candidate_id, 0)
                            > tier
                        )
                        nearest = sorted(refinable, key=residual)[:4]
                        active_refinement_parent_ids = {
                            item.candidate_id for item in nearest
                        }
            place_screen_time_s = time.perf_counter() - place_screen_started
            screen_time_s = grasp_screen_time_s + place_screen_time_s
            if (
                not relation_grasp_candidates
                and callable(collision_reader)
                and place_manifold_input_candidates
            ):
                collision_started = time.perf_counter()
                collision_diagnostics["place"] = collision_reader(
                    place_manifold_input_candidates,
                    ignored_world_objects=contact_ignores,
                )
                collision_diagnostics["diagnostic_time_s"] = (
                    float(collision_diagnostics.get("diagnostic_time_s", 0.0))
                    + time.perf_counter()
                    - collision_started
                )
            preplace_endpoint_summary = endpoint_summary(all_preplace_candidates)
            place_endpoint_summary = endpoint_summary(all_place_candidates)
            relation_screen = {
                "candidate_count": len(task.grasp_candidates),
                "grasp_candidate_count": len(task.grasp_candidates),
                "unique_place_candidate_count": len(declared_place_candidates),
                "place_candidate_count": len(all_place_candidates),
                "eligible_place_candidate_count": len(all_place_candidates),
                "screened_grasp_candidate_count": len(screened_grasp_ids),
                "screened_place_candidate_count": len(screened_place_ids),
                "search_tier": selected_search_tier,
                "grasp_feasible_count": len(raw_grasp_feasible),
                "pregrasp_feasible_count": len(raw_pregrasp_feasible),
                "preplace_feasible_count": len(preplace_feasible),
                "place_feasible_count": len(place_feasible),
                "pick_relation_count": len(grasp_ready_candidates),
                "place_ready_grasp_count": len(place_ready_grasps),
                "complete_relation_count": len(relation_grasp_candidates),
                "refinement_parent_ids": sorted(
                    active_refinement_parent_ids or ()
                ),
                "grasp_screen_time_s": grasp_screen_time_s,
                "grasp_family_screen_time_s": grasp_screen_time_s,
                "place_screen_time_s": place_screen_time_s,
                "place_family_screen_time_s": place_screen_time_s,
                "grasp_endpoint_summary": grasp_endpoint_summary,
                "pregrasp_endpoint_summary": pregrasp_endpoint_summary,
                "preplace_endpoint_summary": preplace_endpoint_summary,
                "place_endpoint_summary": place_endpoint_summary,
                "endpoint_collision_diagnostics": collision_diagnostics,
                "axis_constrained_resolution": axis_resolution,
                "place_manifold_resolution": place_manifold_resolution,
                "screen_time_s": screen_time_s,
                "screen_total_time_s": screen_time_s,
                "candidate_build_time_s": task.candidate_build_time_s,
                **coarse_ik_metrics,
            }
            if coarse_screening_active:
                end_coarse()
                coarse_screening_active = False
            if callable(set_gripper_collision_state):
                set_gripper_collision_state(False)
            if not relation_grasp_candidates:
                primary = PickPlaceRunResult(
                    False,
                    (),
                    failure_stage="relation_screen",
                    message="no grasp relation has feasible preplace and place endpoints",
                    diagnostics={"relation_screen": relation_screen},
                )
                if axis_requested:
                    return self._run_axis_fallback(task, primary, axis_resolver)
                return primary
            grasp_scores = {
                item.candidate_id: float(item.score)
                + max(
                    (
                        0.1 * float(place.score)
                        for place in complete_places_by_grasp[item.candidate_id]
                    ),
                    default=float("-inf"),
                )
                for item in relation_grasp_candidates
            }
            primary = self._run_segmented_chain(
                task,
                relation_grasp_candidates,
                complete_places_by_grasp,
                grasp_scores,
                relation_screen,
            )
            if (
                not primary.success
                and primary.failure_stage == "segmented_chain"
                and axis_requested
            ):
                return self._run_axis_fallback(task, primary, axis_resolver)
            return primary
        finally:
            if coarse_screening_active:
                end_coarse()
            if callable(set_gripper_collision_state):
                set_gripper_collision_state(False)
