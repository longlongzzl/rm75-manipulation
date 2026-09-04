from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from rm75_app.orchestration.multi_object_executor import (
    SceneObjectState,
    TaskSceneState,
)
from rm75_app.pickplace.coordinator import PickPlaceTask
from rm75_app.planning.contracts import (
    BatchPlanningRequest,
    CollisionObject,
    JointConfiguration,
    PlanningScene,
    Pose,
    PoseCandidate,
)
from rm75_app.scenarios.magnetic.backend import (
    MagneticContactPlanningBackend,
    MagneticPickPlaceTaskBuilder,
)
from rm75_app.tasks.manipulation_plan import (
    ManipulationAtom,
    ManipulationPrimitive,
)


def _pose(x: float = 0.0) -> np.ndarray:
    value = np.eye(4)
    value[0, 3] = x
    return value


def _candidate(identifier: str) -> PoseCandidate:
    return PoseCandidate(identifier, Pose([0.0, 0.0, 0.1], [1, 0, 0, 0]))


def test_magnetic_task_builder_propagates_all_support_instance_ids() -> None:
    grasp = _candidate("grasp")
    place = _candidate("place")
    task = PickPlaceTask(
        "child",
        JointConfiguration(("j",), [0.0]),
        (grasp,),
        (place,),
        PlanningScene(
            (
                CollisionObject("child", "sphere", Pose([0, 0, 0], [1, 0, 0, 0]), radius=0.01),
                CollisionObject("left", "sphere", Pose([0, 0, 0], [1, 0, 0, 0]), radius=0.01),
                CollisionObject("right", "sphere", Pose([0, 0, 0], [1, 0, 0, 0]), radius=0.01),
            )
        ),
        place_candidates_by_grasp={"grasp": (place,)},
    )

    class Base:
        def __call__(self, atom, scene):
            del atom, scene
            return task

    atom = ManipulationAtom(
        "a",
        ManipulationPrimitive.SNAP_PLACE,
        "child",
        "panel",
        _pose(),
        metadata={
            "magnetic_support_object_ids": ["left", "right"],
            "magnetic_connection_id": "slot",
        },
    )
    scene = TaskSceneState(
        {
            "child": SceneObjectState("child", "panel", _pose()),
            "left": SceneObjectState("left", "panel", _pose(-0.02)),
            "right": SceneObjectState("right", "panel", _pose(0.02)),
        }
    )

    result = MagneticPickPlaceTaskBuilder(Base())(atom, scene)

    assert result.place_candidates[0].metadata[
        "magnetic_support_object_ids"
    ] == ["left", "right"]
    assert result.places_for_grasp("grasp")[0].metadata[
        "magnetic_connection_id"
    ] == "slot"


def test_magnetic_backend_extends_endpoint_ignore_set_to_both_supports() -> None:
    candidate = _candidate("place")
    candidate = PoseCandidate(
        candidate.candidate_id,
        candidate.pose,
        metadata={"magnetic_support_object_ids": ["left", "right"]},
    )

    class Backend:
        def __init__(self) -> None:
            self.calls = []

        def prepare_pose_candidates_coarse(self, candidates, scene, **kwargs):
            del candidates, scene
            self.calls.append(kwargs)

    backend = Backend()
    wrapper = MagneticContactPlanningBackend(backend)
    wrapper.prepare_pose_candidates_coarse(
        (candidate,),
        PlanningScene(),
        ignore_object_names=("child",),
    )

    assert backend.calls[0]["ignore_object_name"] is None
    assert backend.calls[0]["ignore_object_names"] == (
        "child",
        "left",
        "right",
    )


def test_magnetic_backend_filters_only_supports_for_contact_segment() -> None:
    candidate = PoseCandidate(
        "place",
        Pose([0.0, 0.0, 0.1], [1, 0, 0, 0]),
        metadata={"magnetic_support_object_ids": ["left", "right"]},
    )
    scene = PlanningScene(
        (
            CollisionObject("child", "sphere", Pose([0, 0, 0], [1, 0, 0, 0]), radius=0.01),
            CollisionObject("left", "sphere", Pose([0, 0, 0], [1, 0, 0, 0]), radius=0.01),
            CollisionObject("right", "sphere", Pose([0, 0, 0], [1, 0, 0, 0]), radius=0.01),
            CollisionObject("unrelated", "sphere", Pose([0, 0, 0], [1, 0, 0, 0]), radius=0.01),
        ),
        revision="full",
    )

    class Backend:
        def __init__(self) -> None:
            self.request = None
            self.kwargs = None

        def plan_linear_candidates(self, request, **kwargs):
            self.request = request
            self.kwargs = kwargs
            return "result"

    backend = Backend()
    wrapper = MagneticContactPlanningBackend(backend)
    request = BatchPlanningRequest(
        JointConfiguration(("j",), [0.0]),
        (candidate,),
        scene,
    )

    result = wrapper.plan_linear_candidates(
        request,
        ignore_object_name="left",
    )

    assert result == "result"
    assert {item.name for item in backend.request.scene.objects} == {
        "child",
        "unrelated",
    }
    assert backend.kwargs["ignore_object_name"] is None
    assert backend.kwargs["disable_collision_links"] is None
