from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz
from rm75_app.pickplace.coordinator import (
    PickPlaceCoordinator,
    PickPlaceTask,
    _approach_offset_candidates,
    _place_approach_candidates,
    _pose_matrix,
    _rebuild_places_for_resolved_grasp,
    _target_object_pose,
)
from rm75_app.perception.held_object_refinement import HeldObjectRefinementUpdate
from rm75_app.planning.backends.curobo2 import (
    Curobo2Backend,
    Curobo2BackendConfig,
    _nearest_periodic_joint_positions,
)
from rm75_app.planning.contracts import (
    BatchPlanningResult,
    CandidatePlan,
    CollisionObject,
    JointConfiguration,
    JointTrajectory,
    PlanningScene,
    Pose,
    PoseCandidate,
)


JOINTS = ("j1", "j2")


def test_periodic_joint_goal_uses_nearest_legal_equivalent() -> None:
    targets = np.asarray([[0.4, -4.10], [-0.3, 5.8]])
    reference = np.asarray([0.0, 1.047])
    lower = np.asarray([-3.1, -2.0 * np.pi])
    upper = np.asarray([3.1, 2.0 * np.pi])

    adjusted = _nearest_periodic_joint_positions(targets, reference, lower, upper)

    np.testing.assert_allclose(adjusted[:, 0], targets[:, 0])
    np.testing.assert_allclose(adjusted[0, 1], -4.10 + 2.0 * np.pi)
    np.testing.assert_allclose(adjusted[1, 1], 5.8 - 2.0 * np.pi)


def test_retreat_contact_policy_allows_only_shallow_attached_contacts() -> None:
    pose = Pose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
    backend = Curobo2Backend.__new__(Curobo2Backend)
    backend.config = Curobo2BackendConfig()
    backend._scene = PlanningScene(
        (
            CollisionObject(
                "table", "cuboid", pose, dimensions=[1.0, 1.0, 0.1],
                metadata={"support_surface_kind": "tabletop"},
            ),
            CollisionObject(
                "other_object", "cuboid", pose, dimensions=[0.1, 0.1, 0.1]
            ),
            CollisionObject(
                "deep_support", "cuboid", pose, dimensions=[1.0, 1.0, 0.1],
                metadata={"support_surface_kind": "tabletop"},
            ),
        )
    )
    backend._obstacle_enabled = lambda name: True
    backend._collision_diagnostics_for_states = lambda *args, **kwargs: [
        {
            "collision_type": "world", "world_object": "table",
            "robot_link": "attached_object", "penetration_m": 0.008,
        },
        {
            "collision_type": "world", "world_object": "table",
            "robot_link": "right_pad", "penetration_m": 0.004,
        },
        {
            "collision_type": "world", "world_object": "other_object",
            "robot_link": "attached_object", "penetration_m": 0.004,
        },
        {
            "collision_type": "world", "world_object": "deep_support",
            "robot_link": "attached_object", "penetration_m": 0.03,
        },
    ]

    allowed, _ = backend._attached_start_contact_objects(None, None)

    assert allowed == {"table", "other_object"}


def test_pregrasp_offset_follows_tilted_tcp_approach_axis() -> None:
    tilt = np.deg2rad(40.0)
    rotation = np.asarray(
        [
            [np.cos(tilt), 0.0, -np.sin(tilt)],
            [0.0, 1.0, 0.0],
            [np.sin(tilt), 0.0, np.cos(tilt)],
        ]
    )
    candidate = PoseCandidate(
        "tilted",
        Pose([0.3, 0.1, 0.05], matrix_to_quaternion_wxyz(rotation)),
    )

    pregrasp = _approach_offset_candidates((candidate,), 0.08)[0]

    expected_delta = -0.08 * rotation[:, 2]
    np.testing.assert_allclose(
        pregrasp.pose.position - candidate.pose.position,
        expected_delta,
        atol=1e-8,
    )
    assert abs(float(expected_delta[0])) > 0.04


def test_place_approach_never_shortens_below_object_aware_retreat() -> None:
    candidate = PoseCandidate(
        "tall_release",
        Pose([0.3, 0.1, 0.09], [1.0, 0.0, 0.0, 0.0]),
        metadata={"required_retreat_clearance_m": 0.115},
    )

    approaches = _place_approach_candidates(candidate, 0.08)

    assert approaches
    assert all(
        float(item.metadata["preplace_clearance_m"]) >= 0.115 - 1.0e-9
        for item in approaches
    )
    assert all(
        item.pose.position[2] >= candidate.pose.position[2] + 0.115 - 1.0e-9
        for item in approaches
        if item.candidate_id.startswith("preplace_world_z")
    )


def test_resolved_grasp_rebuild_preserves_held_object_and_target_pose() -> None:
    original = PoseCandidate(
        "grasp",
        Pose([0.4, -0.1, 0.08], [1.0, 0.0, 0.0, 0.0]),
    )
    angle = np.deg2rad(37.0)
    resolved_rotation = np.asarray(
        [
            [np.cos(angle), 0.0, np.sin(angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(angle), 0.0, np.cos(angle)],
        ]
    )
    resolved = PoseCandidate(
        "grasp",
        Pose(
            original.pose.position,
            matrix_to_quaternion_wxyz(resolved_rotation),
        ),
        metadata={"axis_constrained_resolved": True},
    )
    T_tcp_object = np.eye(4)
    T_tcp_object[:3, 3] = [0.0, 0.0, 0.02]
    T_target_object = np.eye(4)
    T_target_object[:3, 3] = [0.2, 0.3, 0.1]
    place = PoseCandidate(
        "place",
        Pose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
        metadata={
            "T_tcp_object": T_tcp_object.tolist(),
            "planning_target_object_pose": T_target_object.tolist(),
        },
    )

    rebuilt = _rebuild_places_for_resolved_grasp(
        original, resolved, (place,)
    )[0]
    new_relation = np.asarray(rebuilt.metadata["T_tcp_object"])

    np.testing.assert_allclose(
        _pose_matrix(resolved.pose) @ new_relation,
        _pose_matrix(original.pose) @ T_tcp_object,
        atol=1e-8,
    )
    np.testing.assert_allclose(
        _pose_matrix(rebuilt.pose) @ new_relation,
        T_target_object,
        atol=1e-8,
    )


def trajectory(start: float, end: float) -> JointTrajectory:
    return JointTrajectory(JOINTS, np.asarray([[start, start], [end, end]]))


class FakePlanner:
    name = "fake"

    def __init__(self) -> None:
        self.events: list[tuple] = []
        self.next_value = 2.0
        self.linear_fail_ids = {"grasp_bad"}

    def update_scene(self, scene) -> None:
        self.events.append(("scene", scene.revision))

    def plan_candidates(self, request):
        identifiers = tuple(item.candidate_id for item in request.candidates)
        self.events.append(("plan", identifiers, tuple(request.current.positions)))
        value = self.next_value
        self.next_value += 1.0
        start = float(request.current.positions[0])
        return BatchPlanningResult(
            tuple(
                CandidatePlan(
                    item.candidate_id, True, trajectory(start, value)
                )
                for item in request.candidates
            ),
            self.name,
        )

    def plan_linear_candidates(self, request, **kwargs):
        identifiers = tuple(item.candidate_id for item in request.candidates)
        self.events.append(
            ("linear", identifiers, tuple(request.current.positions), dict(kwargs))
        )
        value = self.next_value
        self.next_value += 1.0
        start = float(request.current.positions[0])
        return BatchPlanningResult(
            tuple(
                CandidatePlan(
                    item.candidate_id,
                    item.candidate_id not in self.linear_fail_ids,
                    trajectory=(
                        trajectory(start, value)
                        if item.candidate_id not in self.linear_fail_ids
                        else None
                    ),
                    status=(
                        "linear_success"
                        if item.candidate_id not in self.linear_fail_ids
                        else "linear_collision"
                    ),
                )
                for item in request.candidates
            ),
            self.name,
        )

    def attach_object(self, object_name, grasp):
        self.events.append(("attach", object_name, tuple(grasp.positions)))

    def update_attached_object_pose(self, object_name, current, T_tcp_object):
        self.events.append(
            (
                "update_attachment",
                object_name,
                tuple(current.positions),
                tuple(np.asarray(T_tcp_object)[:3, 3]),
            )
        )

    def detach_object(self, object_name, released_pose=None):
        self.events.append(("detach", object_name, released_pose is not None))

    def enable_object_collision(self, object_name):
        self.events.append(("enable", object_name))

    def close(self):
        pass


class FakeExecutor:
    def __init__(self) -> None:
        self.events: list[tuple] = []

    def execute_trajectory(self, stage, trajectory):
        self.events.append((stage, tuple(trajectory.positions[-1])))

    def set_gripper(self, closed):
        self.events.append(("gripper", bool(closed)))


def test_coordinator_keeps_attachment_boundary_and_state_chain() -> None:
    planner = FakePlanner()
    executor = FakeExecutor()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    scene = PlanningScene(
        (CollisionObject("carrot", "cuboid", pose, dimensions=[0.15, 0.03, 0.03]),),
        revision="cached-scene",
    )
    task = PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(
            PoseCandidate("grasp_bad", pose, score=1.0),
            PoseCandidate("grasp_good", pose, score=0.8),
        ),
        place_candidates=(PoseCandidate("bin_a", pose, score=1.0),),
        scene=scene,
    )

    result = PickPlaceCoordinator(planner, executor).run(task)

    assert result.success
    assert [stage.name for stage in result.stages] == [
        "approach",
        "grasp",
        "lift",
        "preplace",
        "place",
        "retreat",
    ]
    assert next(event for event in planner.events if event[0] == "attach")[0] == "attach"
    assert ("detach", "carrot", True) in planner.events
    assert planner.events[-1] == ("enable", "carrot")
    assert [event[0] for event in executor.events] == [
        "approach",
        "grasp",
        "gripper",
        "lift",
        "preplace",
        "place",
        "gripper",
        "retreat",
    ]
    plan_starts = [event[2] for event in planner.events if event[0] == "plan"]
    assert plan_starts == [(0.0, 0.0), (0.0, 0.0), (6.0, 6.0)]
    linear_ids = [event[1] for event in planner.events if event[0] == "linear"]
    assert linear_ids == [
        ("grasp_bad",),
        ("grasp_good",),
        ("lift_world_z:grasp_good",),
        ("bin_a",),
    ]
    assert "plan_grasps" not in [event[0] for event in planner.events]
    lift_linear = next(
        event for event in planner.events
        if event[0] == "linear" and event[1] == ("lift_world_z:grasp_good",)
    )
    assert lift_linear[3]["allow_start_contact_escape"]
    assert lift_linear[3]["disable_collision_links"] is None
    assert executor.events[-1] == ("retreat", (7.0, 7.0))
    place_linear = next(
        event for event in planner.events
        if event[0] == "linear" and event[1] == ("bin_a",)
    )
    assert place_linear[3]["disable_collision_links"] is None


def test_primary_only_grasp_success_does_not_invoke_reverse_probe() -> None:
    planner = FakePlanner()

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(_simple_task())

    assert result.success
    grasp_events = [
        event
        for event in planner.events
        if event[0] == "linear" and event[1] == ("grasp_good",)
    ]
    assert len(grasp_events) == 1
    assert not grasp_events[0][3]["project_distance_to_goal"]
    assert result.diagnostics["grasp_reverse_fallback_used"] is False
    assert result.diagnostics["grasp_fallback_mode"] == "primary_only"


def test_primary_only_grasp_failure_does_not_invoke_reverse_probe() -> None:
    class RetryPlanner(FakePlanner):
        def __init__(self) -> None:
            super().__init__()
            self.linear_fail_ids = set()

        def plan_linear_candidates(self, request, **kwargs):
            if (
                request.candidates[0].candidate_id == "grasp_good"
                and not kwargs["project_distance_to_goal"]
            ):
                identifiers = tuple(item.candidate_id for item in request.candidates)
                self.events.append(
                    ("linear", identifiers, tuple(request.current.positions), dict(kwargs))
                )
                return BatchPlanningResult(
                    (CandidatePlan("grasp_good", False, status="primary_failed"),),
                    self.name,
                )
            return super().plan_linear_candidates(request, **kwargs)

    planner = RetryPlanner()
    executor = FakeExecutor()
    result = PickPlaceCoordinator(planner, executor).run(_simple_task())

    assert not result.success
    grasp_events = [
        event
        for event in planner.events
        if event[0] == "linear" and event[1] == ("grasp_good",)
    ]
    assert len(grasp_events) == 1
    assert not grasp_events[0][3]["project_distance_to_goal"]
    assert not any(event[1] == ("pregrasp:grasp_good",) for event in planner.events if event[0] == "linear")


def test_primary_failures_are_distinct_and_next_candidate_runs() -> None:
    planner = FakePlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    task = replace(
        _simple_task(),
        grasp_candidates=(
            PoseCandidate("grasp_bad", pose, score=1.0),
            PoseCandidate("grasp_good", pose, score=0.8),
        ),
    )

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(task)

    assert result.success
    grasp_events = [
        event[1]
        for event in planner.events
        if event[0] == "linear" and event[1][0].startswith("grasp_")
    ]
    assert grasp_events[:2] == [
        ("grasp_bad",),
        ("grasp_good",),
    ]

    failed_result = PickPlaceCoordinator(
        FakePlanner(), FakeExecutor()
    ).run(replace(_simple_task(), grasp_candidates=(PoseCandidate("grasp_bad", pose),)))
    assert not failed_result.success
    assert [item["stage"] for item in failed_result.diagnostics["candidate_failures"]] == [
        "grasp",
    ]


def test_reverse_grasp_probe_reverses_cached_endpoint_path_and_keeps_sequence() -> None:
    class ReverseProbePlanner(FakePlanner):
        def configuration_for_pose_candidate(self, candidate, names, reference):
            assert tuple(names) == JOINTS
            if candidate.candidate_id != "grasp_good":
                return None
            self.events.append(("cached_grasp", tuple(reference.positions)))
            return JointConfiguration(JOINTS, [50.0, 50.0])

        def plan_linear_candidates(self, request, **kwargs):
            candidate = request.candidates[0]
            self.events.append(
                ("linear", (candidate.candidate_id,), tuple(request.current.positions), dict(kwargs))
            )
            if candidate.candidate_id == "grasp_good":
                return BatchPlanningResult(
                    (CandidatePlan("grasp_good", False, status="forward_failed"),),
                    self.name,
                )
            if candidate.candidate_id == "pregrasp:grasp_good":
                return BatchPlanningResult(
                    (CandidatePlan(
                        candidate.candidate_id,
                        True,
                        trajectory=trajectory(50.0, 2.0),
                        status="reverse_probe_success",
                    ),),
                    self.name,
                )
            return super().plan_linear_candidates(request, **kwargs)

    planner = ReverseProbePlanner()
    executor = FakeExecutor()
    result = PickPlaceCoordinator(
        planner, executor, grasp_fallback_mode="reverse_probe_experimental"
    ).run(_simple_task())

    assert result.success
    assert result.diagnostics["grasp_reverse_fallback_used"] is True
    assert result.diagnostics["grasp_reverse_start_gap_rad"] == 0.0
    assert result.diagnostics["grasp_reverse_probe_status"] == "reverse_probe_success"
    reverse_event = next(
        event for event in planner.events
        if event[0] == "linear" and event[1] == ("pregrasp:grasp_good",)
    )
    assert reverse_event[2] == (50.0, 50.0)
    assert reverse_event[3]["project_distance_to_goal"] is True
    assert reverse_event[3]["ignore_object_name"] == "carrot"
    assert reverse_event[3]["disable_collision_links"] is None
    assert reverse_event[3]["allow_start_contact_escape"] is False
    grasp_stage = next(stage for stage in result.stages if stage.name == "grasp")
    assert tuple(grasp_stage.end.positions) == (50.0, 50.0)
    assert [event[0] for event in executor.events] == [
        "approach", "grasp", "gripper", "lift", "preplace", "place", "gripper", "retreat"
    ]


def test_reverse_grasp_probe_rejects_discontinuous_path_without_new_ik() -> None:
    class DiscontinuousReversePlanner(FakePlanner):
        def configuration_for_pose_candidate(self, candidate, names, reference):
            del candidate, names, reference
            return JointConfiguration(JOINTS, [50.0, 50.0])

        def plan_linear_candidates(self, request, **kwargs):
            candidate = request.candidates[0]
            if candidate.candidate_id == "grasp_bad":
                return BatchPlanningResult(
                    (CandidatePlan("grasp_bad", False, status="forward_failed"),), self.name
                )
            if candidate.candidate_id == "pregrasp:grasp_bad":
                return BatchPlanningResult(
                    (CandidatePlan(candidate.candidate_id, True, trajectory=trajectory(50.0, 3.0)),),
                    self.name,
                )
            return super().plan_linear_candidates(request, **kwargs)

    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    result = PickPlaceCoordinator(
        DiscontinuousReversePlanner(),
        FakeExecutor(),
        grasp_fallback_mode="reverse_probe_experimental",
    ).run(replace(_simple_task(), grasp_candidates=(PoseCandidate("grasp_bad", pose),)))

    assert not result.success
    reverse_failure = result.diagnostics["candidate_failures"][-1]
    assert reverse_failure["stage"] == "grasp_reverse_probe"
    assert reverse_failure["candidates"][0]["status"] == "trajectory_discontinuity"
    assert reverse_failure["candidates"][0]["start_gap_rad"] > 0.10


def test_reverse_probe_without_cached_grasp_does_not_invent_endpoint_ik() -> None:
    planner = FakePlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])

    result = PickPlaceCoordinator(
        planner,
        FakeExecutor(),
        grasp_fallback_mode="reverse_probe_experimental",
    ).run(replace(_simple_task(), grasp_candidates=(PoseCandidate("grasp_bad", pose),)))

    assert not result.success
    assert [item["stage"] for item in result.diagnostics["candidate_failures"]] == [
        "grasp",
        "grasp_reverse_probe",
    ]
    reverse_failure = result.diagnostics["candidate_failures"][-1]
    assert reverse_failure["candidates"][0]["status"] == "cached_endpoint_ik_not_available"
    assert not any(
        event[0] == "linear" and event[1] == ("pregrasp:grasp_bad",)
        for event in planner.events
    )


def test_lift_falls_back_to_reverse_tool_axis_after_world_z_failure() -> None:
    planner = FakePlanner()
    planner.linear_fail_ids.add("lift_world_z:grasp_good")

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(_simple_task())

    assert result.success
    lift_events = [
        event
        for event in planner.events
        if event[0] == "linear" and event[1][0].startswith("lift_")
    ]
    assert [event[1] for event in lift_events] == [
        ("lift_world_z:grasp_good",),
        ("lift_tool_z:grasp_good",),
    ]
    assert not lift_events[0][3]["project_distance_to_goal"]
    assert lift_events[1][3]["project_distance_to_goal"]
    assert lift_events[1][3]["allow_start_contact_escape"]
    assert lift_events[1][3]["disable_collision_links"] is None


class FakeHeldObjectRefiner:
    def __init__(self, accepted: bool = True) -> None:
        self.accepted = accepted
        self.calls: list[tuple[str, tuple[float, ...]]] = []

    def refine_after_lift(self, object_name, current):
        self.calls.append((object_name, tuple(current.positions)))
        transform = np.eye(4)
        transform[:3, 3] = [0.01, -0.002, 0.08]
        return HeldObjectRefinementUpdate(
            self.accepted,
            object_name,
            transform if self.accepted else None,
            0.01,
            1.0,
            "accepted" if self.accepted else "translation_gate",
            source="fake_wrist",
        )


def _simple_task() -> PickPlaceTask:
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    return PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(PoseCandidate("grasp_good", pose),),
        place_candidates=(PoseCandidate("bin_a", pose),),
        scene=PlanningScene(
            (
                CollisionObject(
                    "carrot", "cuboid", pose, dimensions=[0.15, 0.03, 0.03]
                ),
            )
        ),
    )


def test_accepted_held_object_refinement_updates_attachment_after_lift() -> None:
    planner = FakePlanner()
    refiner = FakeHeldObjectRefiner()

    result = PickPlaceCoordinator(
        planner, FakeExecutor(), held_object_refiner=refiner
    ).run(_simple_task())

    assert result.success
    assert result.held_object_refinement is not None
    assert result.held_object_refinement.accepted
    assert refiner.calls == [("carrot", (4.0, 4.0))]
    event_names = [event[0] for event in planner.events]
    update_index = event_names.index("update_attachment")
    assert event_names[update_index - 1] == "attach"
    assert event_names[update_index + 1] == "detach"
    assert "plan_grasps" not in event_names


def test_rejected_held_object_refinement_keeps_original_attachment() -> None:
    planner = FakePlanner()
    refiner = FakeHeldObjectRefiner(accepted=False)

    result = PickPlaceCoordinator(
        planner, FakeExecutor(), held_object_refiner=refiner
    ).run(_simple_task())

    assert result.success
    assert result.held_object_refinement is not None
    assert not result.held_object_refinement.accepted
    assert "update_attachment" not in [event[0] for event in planner.events]


def test_selected_grasp_uses_only_its_paired_place_candidates() -> None:
    planner = FakePlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    paired = PoseCandidate("paired_place", pose)
    unrelated = PoseCandidate("unrelated_place", pose)
    task = PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(
            PoseCandidate("grasp_bad", pose),
            PoseCandidate("grasp_good", pose),
        ),
        place_candidates=(unrelated, paired),
        place_candidates_by_grasp={
            "grasp_bad": (unrelated,),
            "grasp_good": (paired,),
        },
        scene=PlanningScene(
            (CollisionObject("carrot", "cuboid", pose, dimensions=[0.15, 0.03, 0.03]),)
        ),
    )
    result = PickPlaceCoordinator(planner, FakeExecutor()).run(task)
    assert result.success
    assert result.selected_grasp == "grasp_good"
    assert result.selected_place == "paired_place"
    linear_ids = [event[1] for event in planner.events if event[0] == "linear"]
    assert ("paired_place",) in linear_ids
    assert ("unrelated_place",) not in linear_ids
    assert "plan_grasps" not in [event[0] for event in planner.events]


def test_release_updates_scene_with_object_pose_not_tcp_pose() -> None:
    planner = FakePlanner()
    tcp_pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    object_transform = np.eye(4)
    object_transform[:3, 3] = [0.3, -0.1, 0.05]
    place = PoseCandidate(
        "place",
        tcp_pose,
        metadata={"target_object_pose": object_transform.tolist()},
    )
    task = PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(PoseCandidate("grasp_good", tcp_pose),),
        place_candidates=(place,),
        scene=PlanningScene(
            (CollisionObject("carrot", "cuboid", tcp_pose, dimensions=[0.1, 0.03, 0.03]),)
        ),
    )

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(task)

    assert result.success
    detaches = [event for event in planner.events if event[0] == "detach"]
    assert detaches[-1] == ("detach", "carrot", True)
    np.testing.assert_allclose(_target_object_pose(place).position, object_transform[:3, 3])


def test_failed_grasp_preserves_structured_collision_diagnostics() -> None:
    planner = FakePlanner()

    def fail_linear(request, **kwargs):
        del kwargs
        return BatchPlanningResult(
            tuple(
                CandidatePlan(
                    candidate.candidate_id,
                    False,
                    status="Start or End state in collision",
                    diagnostics={
                        "collision_diagnostics": [
                            {
                                "collision_type": "world",
                                "state": "grasp_goal",
                                "robot_link": "link6",
                                "world_object": "tennis",
                                "penetration_m": 0.003,
                            }
                        ]
                    },
                )
                for candidate in request.candidates
            ),
            planner.name,
        )

    planner.plan_linear_candidates = fail_linear
    result = PickPlaceCoordinator(planner, FakeExecutor()).run(_simple_task())

    assert not result.success
    failures = result.diagnostics["candidate_failures"]
    collision = failures[0]["candidates"][0]["collision_diagnostics"][0]
    assert collision["robot_link"] == "link6"
    assert collision["world_object"] == "tennis"


def test_four_endpoint_relation_screen_selects_complete_lower_score_chain() -> None:
    class RelationPlanner(FakePlanner):
        def __init__(self):
            super().__init__()
            self.prepared = []

        def prepare_pose_candidates(self, candidates, scene, **kwargs):
            del scene
            self.prepared.append(
                (
                    tuple(item.candidate_id for item in candidates),
                    tuple(kwargs["ignore_object_names"]),
                )
            )

        def feasible_pose_candidate_ids(self, candidates):
                return frozenset(
                    item.candidate_id
                    for item in candidates
                    if "low_place" in item.candidate_id
                    or "grasp_low" in item.candidate_id
            )


    planner = RelationPlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    task = PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(
            PoseCandidate("grasp_high", pose, score=1.0),
            PoseCandidate("grasp_low", pose, score=0.5),
        ),
        place_candidates=(
            PoseCandidate("high_place", pose),
            PoseCandidate("low_place", pose),
        ),
        place_candidates_by_grasp={
            "grasp_high": (PoseCandidate("high_place", pose),),
            "grasp_low": (PoseCandidate("low_place", pose),),
        },
        scene=PlanningScene(
            (CollisionObject("carrot", "cuboid", pose, dimensions=[0.1, 0.03, 0.03]),)
        ),
    )

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(task)

    assert result.success
    assert result.selected_grasp == "grasp_low"
    assert result.selected_place == "low_place"
    assert planner.prepared[0] == (
        (
            "pregrasp:grasp_high",
            "pregrasp:grasp_low",
            "grasp_high",
            "grasp_low",
        ),
        ("carrot",),
    )
    assert planner.prepared[1] == (
        (
            "preplace_world_z:low_place",
            "preplace_tool_z:low_place",
            "preplace_world_z_75mm:low_place",
            "preplace_tool_z_75mm:low_place",
            "preplace_world_z_50mm:low_place",
            "preplace_tool_z_50mm:low_place",
            "preplace_world_z_30mm:low_place",
            "preplace_tool_z_30mm:low_place",
            "low_place",
        ),
        ("carrot",),
    )
    linear_ids = [event[1] for event in planner.events if event[0] == "linear"]
    assert ("grasp_low",) in linear_ids
    assert ("low_place",) in linear_ids
    assert ("grasp_high",) not in linear_ids


def test_batch64_grasp_failure_does_not_invoke_slow_fallback() -> None:
    class CoarseOnlyPlanner(FakePlanner):
        def __init__(self):
            super().__init__()
            self.feasible_ids: set[str] = set()
            self.prepare_events: list[tuple[str, tuple[str, ...]]] = []

        def prepare_pose_candidates_coarse(self, candidates, scene, **kwargs):
            del scene, kwargs
            identifiers = tuple(item.candidate_id for item in candidates)
            self.prepare_events.append(("coarse", identifiers))
            if all("place" in item for item in identifiers):
                self.feasible_ids.update(identifiers)

        def prepare_pose_candidates(self, candidates, scene, **kwargs):
            del scene, kwargs
            identifiers = tuple(item.candidate_id for item in candidates)
            self.prepare_events.append(("fallback", identifiers))

        def feasible_pose_candidate_ids(self, candidates):
            return frozenset(
                item.candidate_id
                for item in candidates
                if item.candidate_id in self.feasible_ids
            )

    planner = CoarseOnlyPlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    grasp = PoseCandidate("grasp_stable", pose)
    place = PoseCandidate("place_ready", pose)
    task = PickPlaceTask(
        object_name="pen",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(grasp,),
        place_candidates=(place,),
        place_candidates_by_grasp={grasp.candidate_id: (place,)},
        scene=PlanningScene(
            (CollisionObject("pen", "cuboid", pose, dimensions=[0.1, 0.01, 0.01]),)
        ),
    )

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(task)

    assert not result.success
    assert ("coarse", ("pregrasp:grasp_stable", "grasp_stable")) in planner.prepare_events
    assert not any(event[0] == "fallback" for event in planner.prepare_events)


def test_relation_screen_reports_merged_coarse_ik_batch_metrics() -> None:
    class InstrumentedPlanner(FakePlanner):
        def __init__(self):
            super().__init__()
            self.config = SimpleNamespace(coarse_ik_batch_size=64)
            self.prepared: list[tuple[tuple[str, ...], tuple[str, ...]]] = []

        def prepare_pose_candidates_coarse(self, candidates, scene, **kwargs):
            del scene
            identifiers = tuple(item.candidate_id for item in candidates)
            self.prepared.append((identifiers, tuple(kwargs["ignore_object_names"])))

        def feasible_pose_candidate_ids(self, candidates):
            return frozenset(item.candidate_id for item in candidates)

    planner = InstrumentedPlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    grasp = PoseCandidate("grasp", pose)
    place = PoseCandidate("place", pose)
    task = PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(grasp,),
        place_candidates=(place,),
        place_candidates_by_grasp={grasp.candidate_id: (place,)},
        scene=PlanningScene(
            (CollisionObject("carrot", "cuboid", pose, dimensions=[0.1, 0.03, 0.03]),)
        ),
        candidate_build_time_s=0.125,
    )

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(task)

    assert result.success
    assert planner.prepared[0] == (
        ("pregrasp:grasp", "grasp"),
        ("carrot",),
    )
    assert planner.prepared[1][-1] == ("carrot",)
    diagnostics = result.diagnostics["relation_screen"]
    assert diagnostics["candidate_build_time_s"] == 0.125
    assert diagnostics["coarse_ik_call_count"] == 2
    assert diagnostics["coarse_ik_rows_requested"] == 11
    assert diagnostics["coarse_ik_rows_padded"] == 128
    assert diagnostics["stable_ik_call_count"] == 0
    assert diagnostics["complete_relation_count"] == 1


def test_progressive_preplace_stops_after_nominal_rank_without_duplicate_screening() -> None:
    class ProgressivePlanner(FakePlanner):
        def __init__(self) -> None:
            super().__init__()
            self.prepared: list[tuple[tuple[str, ...], tuple[str, ...]]] = []

        def prepare_pose_candidates_coarse(self, candidates, scene, **kwargs):
            del scene
            self.prepared.append(
                (tuple(item.candidate_id for item in candidates), tuple(kwargs["ignore_object_names"]))
            )

        def feasible_pose_candidate_ids(self, candidates):
            return frozenset(item.candidate_id for item in candidates)

    planner = ProgressivePlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    grasp = PoseCandidate("grasp", pose)
    place = PoseCandidate("place", pose)
    task = PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(grasp,),
        place_candidates=(place,),
        place_candidates_by_grasp={grasp.candidate_id: (place,)},
        scene=PlanningScene((CollisionObject("carrot", "cuboid", pose, dimensions=[0.1, 0.03, 0.03]),)),
    )

    result = PickPlaceCoordinator(
        planner, FakeExecutor(), relation_screen_mode="lazy_place_progressive_preplace"
    ).run(task)

    assert result.success
    prepared_ids = [item for ids, _ignored in planner.prepared for item in ids]
    assert "preplace_world_z:place" in prepared_ids
    assert "preplace_tool_z:place" in prepared_ids
    assert not any("75mm" in item or "50mm" in item or "30mm" in item for item in prepared_ids)
    assert len(prepared_ids) == len(set(prepared_ids))
    assert all(ignored == ("carrot",) for _ids, ignored in planner.prepared)


def test_progressive_preplace_advances_once_to_rank_one_and_retains_fallback_mapping() -> None:
    class FallbackPlanner(FakePlanner):
        def __init__(self) -> None:
            super().__init__()
            self.prepared: list[tuple[str, ...]] = []

        def prepare_pose_candidates_coarse(self, candidates, scene, **kwargs):
            del scene, kwargs
            self.prepared.append(tuple(item.candidate_id for item in candidates))

        def feasible_pose_candidate_ids(self, candidates):
            return frozenset(
                item.candidate_id for item in candidates
                if "preplace" not in item.candidate_id or "75mm" in item.candidate_id
            )

    planner = FallbackPlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    grasp = PoseCandidate("grasp", pose)
    place = PoseCandidate("place", pose)
    task = PickPlaceTask(
        object_name="carrot", current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(grasp,), place_candidates=(place,),
        place_candidates_by_grasp={grasp.candidate_id: (place,)},
        scene=PlanningScene((CollisionObject("carrot", "cuboid", pose, dimensions=[0.1, 0.03, 0.03]),)),
    )
    result = PickPlaceCoordinator(planner, FakeExecutor(), relation_screen_mode="lazy_place_progressive_preplace").run(task)
    assert result.success
    flattened = [candidate for event in planner.prepared for candidate in event]
    assert "preplace_world_z:place" in flattened
    assert "preplace_world_z_75mm:place" in flattened
    assert not any("50mm" in candidate or "30mm" in candidate for candidate in flattened)
    assert len(flattened) == len(set(flattened))


def test_discrete_success_does_not_invoke_axis_fallback() -> None:
    class DiscreteFirstPlanner(FakePlanner):
        def resolve_axis_constrained_pose_candidates(self, *args, **kwargs):
            raise AssertionError("axis fallback must not run after discrete success")

    task = _simple_task()
    task = replace(
        task,
        grasp_candidates=(
            replace(
                task.grasp_candidates[0],
                metadata={"free_rotation_axis_local": "y"},
            ),
        ),
    )

    result = PickPlaceCoordinator(
        DiscreteFirstPlanner(), FakeExecutor()
    ).run(task)

    assert result.success
    assert result.diagnostics["relation_screen"]["axis_constrained_resolution"] == {
        "enabled": True,
        "strategy": "discrete_primary",
        "attempted": False,
        "input_count": 1,
        "resolved_count": 1,
    }
    assert "axis_fallback" not in result.diagnostics


def test_axis_fallback_runs_only_after_discrete_relation_failure() -> None:
    class AxisFallbackPlanner(FakePlanner):
        def __init__(self):
            super().__init__()
            self.axis_calls = 0

        def prepare_pose_candidates_coarse(self, candidates, scene, **kwargs):
            del candidates, scene, kwargs

        def feasible_pose_candidate_ids(self, candidates):
            return frozenset(
                item.candidate_id
                for item in candidates
                if "axis_solution" in item.candidate_id
                or "place" in item.candidate_id
            )

        def resolve_axis_constrained_pose_candidates(
            self, candidates, scene, **kwargs
        ):
            del scene, kwargs
            self.axis_calls += 1
            source = candidates[0]
            return (
                PoseCandidate(
                    f"{source.candidate_id}__axis_solution_01",
                    source.pose,
                    source.score,
                    {
                        **dict(source.metadata),
                        "axis_constrained_resolved": True,
                        "source_grasp_candidate_id": source.candidate_id,
                    },
                ),
            )

    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    grasp = PoseCandidate(
        "grasp",
        pose,
        metadata={"free_rotation_axis_local": "y"},
    )
    place = PoseCandidate("place", pose)
    task = PickPlaceTask(
        object_name="pen",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(grasp,),
        place_candidates=(place,),
        place_candidates_by_grasp={"grasp": (place,)},
        scene=PlanningScene(
            (CollisionObject("pen", "cuboid", pose, dimensions=[0.1, 0.01, 0.01]),)
        ),
    )
    planner = AxisFallbackPlanner()

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(task)

    assert result.success
    assert planner.axis_calls == 1
    assert result.selected_grasp == "grasp__axis_solution_01"
    assert result.diagnostics["planning_strategy"] == "discrete_then_axis_fallback"
    assert result.diagnostics["discrete_primary"]["failure_stage"] == "relation_screen"
    assert result.diagnostics["axis_fallback"] == {
        "attempted": True,
        "input_count": 1,
        "resolved_count": 1,
    }


def test_failed_place_line_falls_back_to_next_matching_relation() -> None:
    class PairPlanner(FakePlanner):
        def __init__(self):
            super().__init__()
            self.linear_fail_ids = {"place_a"}

    planner = PairPlanner()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    place_a = PoseCandidate("place_a", pose, score=1.0)
    place_b = PoseCandidate("place_b", pose, score=0.9)
    task = PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(
            PoseCandidate("grasp_a", pose, score=1.0),
            PoseCandidate("grasp_b", pose, score=0.9),
        ),
        place_candidates=(place_a, place_b),
        place_candidates_by_grasp={
            "grasp_a": (place_a,),
            "grasp_b": (place_b,),
        },
        scene=PlanningScene(
            (CollisionObject("carrot", "cuboid", pose, dimensions=[0.1, 0.03, 0.03]),)
        ),
    )

    result = PickPlaceCoordinator(planner, FakeExecutor()).run(task)

    assert result.success
    assert result.selected_grasp == "grasp_b"
    assert result.selected_place == "place_b"
    linear_ids = [event[1] for event in planner.events if event[0] == "linear"]
    assert ("place_a",) in linear_ids
    assert ("place_b",) in linear_ids
    assert linear_ids.index(("place_a",)) < linear_ids.index(("grasp_b",))
    assert "attach_batch" not in [event[0] for event in planner.events]


def test_failed_direct_pregrasp_retries_through_task_initial_configuration() -> None:
    class InitialFallbackPlanner(FakePlanner):
        def plan_candidates(self, request):
            identifiers = tuple(item.candidate_id for item in request.candidates)
            current = tuple(request.current.positions)
            if identifiers == ("pregrasp:grasp",) and np.allclose(current, [0.5, 0.5]):
                self.events.append(("plan_failed", identifiers, current))
                return BatchPlanningResult(
                    (
                        CandidatePlan(
                            identifiers[0], False, status="trajectory_failed"
                        ),
                    ),
                    self.name,
                )
            return super().plan_candidates(request)

        def plan_to_configuration(
            self, current, target, scene, *, max_attempts=2, candidate_id="joint_configuration"
        ):
            del scene, max_attempts
            self.events.append(
                (
                    "plan_to_configuration",
                    tuple(current.positions),
                    tuple(target.positions),
                )
            )
            return CandidatePlan(
                candidate_id,
                True,
                trajectory=JointTrajectory(
                    JOINTS,
                    np.asarray([current.positions, target.positions]),
                ),
            )

    planner = InitialFallbackPlanner()
    executor = FakeExecutor()
    pose = Pose([0.4, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    grasp = PoseCandidate("grasp", pose)
    place = PoseCandidate("place", pose)
    task = PickPlaceTask(
        object_name="carrot",
        current=JointConfiguration(JOINTS, [0.5, 0.5]),
        pregrasp_fallback_configuration=JointConfiguration(JOINTS, [0.0, 0.0]),
        grasp_candidates=(grasp,),
        place_candidates=(place,),
        place_candidates_by_grasp={"grasp": (place,)},
        scene=PlanningScene(
            (CollisionObject("carrot", "cuboid", pose, dimensions=[0.1, 0.03, 0.03]),)
        ),
    )

    result = PickPlaceCoordinator(planner, executor).run(task)

    assert result.success
    assert result.diagnostics["pregrasp_initial_fallback_used"] is True
    assert [stage.name for stage in result.stages[:2]] == [
        "pregrasp_fallback_initial",
        "approach",
    ]
    assert (
        "plan_to_configuration",
        (0.5, 0.5),
        (0.0, 0.0),
    ) in planner.events
    assert (
        "plan",
        ("pregrasp:grasp",),
        (0.0, 0.0),
    ) in planner.events
