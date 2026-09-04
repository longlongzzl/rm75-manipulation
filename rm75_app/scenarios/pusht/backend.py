"""Adapters that turn one planar Push-T action into a continuous robot program.

The backend plans the complete short push before the robot starts moving:
free-space hover -> vertical approach -> contact -> push -> vertical retract.
Only the T object is removed from the collision world during intentional contact;
all other obstacles and self-collision constraints remain active.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol, Sequence

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


class TrajectorySink(Protocol):
    def execute_trajectory(self, stage: str, trajectory: JointTrajectory) -> None: ...


JointStateProvider = Callable[[], JointConfiguration]
SceneProvider = Callable[[bool], PlanningScene]


@dataclass(frozen=True)
class PushToolConfig:
    """Calibrated pusher geometry and motion limits.

    ``contact_tcp_z_m`` is the TCP height required for the pusher to contact the
    T on the tabletop.  When ``align_tool_x_with_push`` is enabled, local +X is
    aligned with the push direction and local +Z points down, allowing the
    existing constrained linear planner to express any planar push.
    """

    tool_frame: str = "gripper_tcp"
    contact_tcp_z_m: float = 0.055
    hover_height_m: float = 0.060
    maximum_linear_segment_m: float = 0.035
    max_stage_start_gap_rad: float = 0.10
    max_attempts: int = 3
    align_tool_x_with_push: bool = True
    fixed_quaternion_wxyz: tuple[float, float, float, float] = (
        0.0,
        1.0,
        0.0,
        0.0,
    )
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.contact_tcp_z_m <= 0.0:
            raise ValueError("contact_tcp_z_m must be positive")
        if self.hover_height_m <= 0.0:
            raise ValueError("hover_height_m must be positive")
        if self.maximum_linear_segment_m <= 0.0:
            raise ValueError("maximum_linear_segment_m must be positive")
        if self.max_stage_start_gap_rad <= 0.0:
            raise ValueError("max_stage_start_gap_rad must be positive")
        if self.max_attempts < 1:
            raise ValueError("max_attempts must be positive")
        quaternion = np.asarray(self.fixed_quaternion_wxyz, dtype=np.float64)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError("fixed_quaternion_wxyz must be a finite quaternion")
        if float(np.linalg.norm(quaternion)) < 1.0e-8:
            raise ValueError("fixed_quaternion_wxyz must not be zero")
        object.__setattr__(
            self,
            "fixed_quaternion_wxyz",
            tuple((quaternion / np.linalg.norm(quaternion)).tolist()),
        )
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class PlannedPushSegment:
    stage: str
    trajectory: JointTrajectory
    scene_kind: str
    linear_axis: str | None
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


@dataclass(frozen=True)
class PlannedPushProgram:
    segments: tuple[PlannedPushSegment, ...]
    start_configuration: JointConfiguration
    end_configuration: JointConfiguration
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.segments:
            raise ValueError("planned push program must contain at least one segment")
        object.__setattr__(self, "segments", tuple(self.segments))
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


class PushPlanningError(RuntimeError):
    def __init__(
        self,
        stage: str,
        message: str,
        *,
        diagnostics: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(f"{stage}: {message}")
        self.stage = str(stage)
        self.diagnostics = dict(diagnostics or {})


def _unit_xy(value: Sequence[float]) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64).reshape(2)
    if not np.all(np.isfinite(vector)):
        raise ValueError("planar direction must be finite")
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-9:
        raise ValueError("planar direction must not be zero")
    return vector / norm


def _joint_end(trajectory: JointTrajectory) -> JointConfiguration:
    return JointConfiguration(
        tuple(trajectory.joint_names),
        np.asarray(trajectory.positions[-1], dtype=np.float64),
    )


class CuroboWaypointPushBackend:
    """Plan and execute one short Push-T action using existing cuRobo contracts.

    ``scene_provider(False)`` must return the complete free-space planning scene.
    ``scene_provider(True)`` must return an otherwise identical scene with only
    the manipulated T object removed/disabled.  The backend does not infer or
    broaden collision exemptions.
    """

    def __init__(
        self,
        planner: Any,
        trajectory_sink: TrajectorySink,
        joint_state_provider: JointStateProvider,
        scene_provider: SceneProvider,
        *,
        tool: PushToolConfig | None = None,
    ) -> None:
        self.planner = planner
        self.trajectory_sink = trajectory_sink
        self.joint_state_provider = joint_state_provider
        self.scene_provider = scene_provider
        self.tool = tool or PushToolConfig()
        self.last_program: PlannedPushProgram | None = None

    def _quaternion(self, direction_xy: np.ndarray) -> np.ndarray:
        if not self.tool.align_tool_x_with_push:
            return np.asarray(
                self.tool.fixed_quaternion_wxyz,
                dtype=np.float64,
            )
        x_axis = np.asarray(
            [direction_xy[0], direction_xy[1], 0.0],
            dtype=np.float64,
        )
        x_axis /= float(np.linalg.norm(x_axis))
        z_axis = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= float(np.linalg.norm(y_axis))
        rotation = np.stack((x_axis, y_axis, z_axis), axis=1)
        return matrix_to_quaternion_wxyz(rotation)

    def _candidate(
        self,
        identifier: str,
        xy: np.ndarray,
        z: float,
        quaternion: np.ndarray,
    ) -> PoseCandidate:
        return PoseCandidate(
            identifier,
            Pose(
                [float(xy[0]), float(xy[1]), float(z)],
                quaternion,
            ),
            metadata={
                "scenario": "push_t",
                "push_program_stage": identifier,
                **dict(self.tool.metadata),
            },
        )

    @staticmethod
    def _split_line(
        start: np.ndarray,
        end: np.ndarray,
        maximum_segment_m: float,
    ) -> tuple[np.ndarray, ...]:
        delta = np.asarray(end, dtype=np.float64) - np.asarray(start, dtype=np.float64)
        distance = float(np.linalg.norm(delta))
        count = max(1, int(np.ceil(distance / float(maximum_segment_m))))
        return tuple(
            np.asarray(start, dtype=np.float64) + delta * (index / count)
            for index in range(1, count + 1)
        )

    @staticmethod
    def _continuity_gap(
        current: JointConfiguration,
        trajectory: JointTrajectory,
    ) -> float:
        if tuple(current.names) != tuple(trajectory.joint_names):
            return float("inf")
        return float(
            np.max(
                np.abs(
                    np.asarray(current.positions, dtype=np.float64)
                    - np.asarray(trajectory.positions[0], dtype=np.float64)
                )
            )
        )

    def _plan_pose(
        self,
        *,
        stage: str,
        current: JointConfiguration,
        candidate: PoseCandidate,
        scene: PlanningScene,
        prefer_direct: bool,
    ) -> CandidatePlan:
        result = self.planner.plan_candidates(
            BatchPlanningRequest(
                current=current,
                candidates=(candidate,),
                scene=scene,
                tool_frame=self.tool.tool_frame,
                max_attempts=self.tool.max_attempts,
                prefer_unbiased_ik=True,
                prefer_direct_tcp_path=prefer_direct,
            )
        )
        plan = result.best((candidate,))
        if plan is None or not plan.success or plan.trajectory is None:
            diagnostics = {
                "backend": getattr(result, "backend", None),
                "plans": [
                    {
                        "candidate_id": item.candidate_id,
                        "status": item.status,
                        "diagnostics": dict(item.diagnostics),
                    }
                    for item in result.plans
                ],
            }
            raise PushPlanningError(
                stage,
                "pose segment could not be planned",
                diagnostics=diagnostics,
            )
        return plan

    def _plan_linear(
        self,
        *,
        stage: str,
        current: JointConfiguration,
        candidate: PoseCandidate,
        scene: PlanningScene,
        axis: str,
    ) -> CandidatePlan:
        request = BatchPlanningRequest(
            current=current,
            candidates=(candidate,),
            scene=scene,
            tool_frame=self.tool.tool_frame,
            max_attempts=self.tool.max_attempts,
        )
        linear = getattr(self.planner, "plan_linear_candidates", None)
        if callable(linear):
            result = linear(
                request,
                axis=axis,
                project_distance_to_goal=True,
                ignore_object_name=None,
                disable_collision_links=None,
                allow_start_contact_escape=False,
            )
            plan = result.best((candidate,))
        else:
            plan = self._plan_pose(
                stage=stage,
                current=current,
                candidate=candidate,
                scene=scene,
                prefer_direct=True,
            )
            return plan
        if plan is None or not plan.success or plan.trajectory is None:
            raise PushPlanningError(
                stage,
                "linear segment could not be planned",
                diagnostics={
                    "axis": axis,
                    "plans": [
                        {
                            "candidate_id": item.candidate_id,
                            "status": item.status,
                            "diagnostics": dict(item.diagnostics),
                        }
                        for item in result.plans
                    ],
                },
            )
        return plan

    def plan_cartesian_push(
        self,
        *,
        approach_xy: np.ndarray,
        contact_xy: np.ndarray,
        end_xy: np.ndarray,
    ) -> PlannedPushProgram:
        approach_xy = np.asarray(approach_xy, dtype=np.float64).reshape(2)
        contact_xy = np.asarray(contact_xy, dtype=np.float64).reshape(2)
        end_xy = np.asarray(end_xy, dtype=np.float64).reshape(2)
        if not all(
            np.all(np.isfinite(item))
            for item in (approach_xy, contact_xy, end_xy)
        ):
            raise ValueError("Push-T waypoints must be finite")
        push_direction = _unit_xy(end_xy - contact_xy)
        contact_direction = _unit_xy(contact_xy - approach_xy)
        if float(np.dot(push_direction, contact_direction)) < 0.70:
            raise ValueError(
                "approach and push directions differ too much for one continuous contact"
            )
        quaternion = self._quaternion(push_direction)
        contact_z = float(self.tool.contact_tcp_z_m)
        hover_z = contact_z + float(self.tool.hover_height_m)
        free_scene = self.scene_provider(False)
        contact_scene = self.scene_provider(True)
        start = self.joint_state_provider()
        current = start
        segments: list[PlannedPushSegment] = []

        def append(
            stage: str,
            plan: CandidatePlan,
            *,
            scene_kind: str,
            linear_axis: str | None,
        ) -> None:
            nonlocal current
            assert plan.trajectory is not None
            gap = self._continuity_gap(current, plan.trajectory)
            if gap > self.tool.max_stage_start_gap_rad:
                raise PushPlanningError(
                    stage,
                    f"joint discontinuity {gap:.6f} rad exceeds "
                    f"{self.tool.max_stage_start_gap_rad:.6f} rad",
                    diagnostics={"joint_start_gap_rad": gap},
                )
            segments.append(
                PlannedPushSegment(
                    stage,
                    plan.trajectory,
                    scene_kind,
                    linear_axis,
                    diagnostics={
                        "candidate_id": plan.candidate_id,
                        "status": plan.status,
                        "solve_time_s": plan.solve_time,
                        "joint_start_gap_rad": gap,
                        **dict(plan.diagnostics),
                    },
                )
            )
            current = _joint_end(plan.trajectory)

        hover_approach = self._candidate(
            "push_hover_approach",
            approach_xy,
            hover_z,
            quaternion,
        )
        append(
            "push_hover_approach",
            self._plan_pose(
                stage="push_hover_approach",
                current=current,
                candidate=hover_approach,
                scene=free_scene,
                prefer_direct=False,
            ),
            scene_kind="free",
            linear_axis=None,
        )

        approach = self._candidate(
            "push_approach",
            approach_xy,
            contact_z,
            quaternion,
        )
        append(
            "push_descend",
            self._plan_linear(
                stage="push_descend",
                current=current,
                candidate=approach,
                scene=free_scene,
                axis="z",
            ),
            scene_kind="free",
            linear_axis="z",
        )

        contact_points = self._split_line(
            approach_xy,
            contact_xy,
            self.tool.maximum_linear_segment_m,
        )
        for index, point in enumerate(contact_points, start=1):
            candidate = self._candidate(
                f"push_contact_{index:02d}",
                point,
                contact_z,
                quaternion,
            )
            append(
                f"push_contact_{index:02d}",
                self._plan_linear(
                    stage=f"push_contact_{index:02d}",
                    current=current,
                    candidate=candidate,
                    scene=contact_scene,
                    axis="x",
                ),
                scene_kind="contact",
                linear_axis="x",
            )

        push_points = self._split_line(
            contact_xy,
            end_xy,
            self.tool.maximum_linear_segment_m,
        )
        for index, point in enumerate(push_points, start=1):
            candidate = self._candidate(
                f"push_translate_{index:02d}",
                point,
                contact_z,
                quaternion,
            )
            append(
                f"push_translate_{index:02d}",
                self._plan_linear(
                    stage=f"push_translate_{index:02d}",
                    current=current,
                    candidate=candidate,
                    scene=contact_scene,
                    axis="x",
                ),
                scene_kind="contact",
                linear_axis="x",
            )

        retract = self._candidate(
            "push_retract",
            end_xy,
            hover_z,
            quaternion,
        )
        append(
            "push_retract",
            self._plan_linear(
                stage="push_retract",
                current=current,
                candidate=retract,
                scene=contact_scene,
                axis="z",
            ),
            scene_kind="contact",
            linear_axis="z",
        )
        return PlannedPushProgram(
            tuple(segments),
            start,
            current,
            diagnostics={
                "approach_xy": approach_xy.tolist(),
                "contact_xy": contact_xy.tolist(),
                "end_xy": end_xy.tolist(),
                "push_direction_world_xy": push_direction.tolist(),
                "contact_segment_count": len(contact_points),
                "push_segment_count": len(push_points),
                "contact_scene_policy": "omit_only_t_object",
            },
        )

    def execute_program(self, program: PlannedPushProgram) -> None:
        for segment in program.segments:
            self.trajectory_sink.execute_trajectory(
                segment.stage,
                segment.trajectory,
            )

    def execute_cartesian_push(
        self,
        *,
        approach_xy: np.ndarray,
        contact_xy: np.ndarray,
        end_xy: np.ndarray,
        speed_mps: float,
        metadata: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        del speed_mps
        program = self.plan_cartesian_push(
            approach_xy=approach_xy,
            contact_xy=contact_xy,
            end_xy=end_xy,
        )
        self.execute_program(program)
        self.last_program = program
        return {
            "backend": "curobo_waypoint_push",
            "segment_count": len(program.segments),
            "program_diagnostics": dict(program.diagnostics),
            "source_metadata": dict(metadata),
            "segments": [
                {
                    "stage": item.stage,
                    "points": int(len(item.trajectory.positions)),
                    "scene_kind": item.scene_kind,
                    "linear_axis": item.linear_axis,
                    "diagnostics": dict(item.diagnostics),
                }
                for item in program.segments
            ],
        }
