from __future__ import annotations

import numpy as np

from rm75_app.orchestration.multi_object_executor import (
    ObjectLifecycle,
    SceneObjectState,
    TaskSceneState,
)
from rm75_app.perception.rrtrack import RRTrackOutput, TrackerState
from rm75_app.perception.rrtrack.models import Agreement
from rm75_app.scenarios.rrtrack_bridge import (
    RRTrackInstanceSample,
    RRTrackPushTTracker,
    RRTrackSceneAdapter,
)


def _pose(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    c, s = float(np.cos(yaw)), float(np.sin(yaw))
    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    value[:3, 3] = [x, y, z]
    return value


def _output(
    pose: np.ndarray | None,
    *,
    accepted: bool = True,
    state: TrackerState = TrackerState.TRACKING,
    frame_index: int = 10,
) -> RRTrackOutput:
    return RRTrackOutput(
        frame_index=frame_index,
        state=state,
        T_cam_obj=pose,
        mask=np.ones((2, 2), dtype=bool),
        rendered_mask=np.ones((2, 2), dtype=bool),
        agreement=Agreement(0.9, 0.8, 0.1, 4, 4, 4),
        accepted=accepted,
        event="tracked",
    )


def test_scene_adapter_updates_only_accepted_nonheld_instances() -> None:
    scene = TaskSceneState(
        {
            "t:0": SceneObjectState("t:0", "t", _pose(0.0, 0.0)),
            "panel:0": SceneObjectState(
                "panel:0",
                "panel",
                _pose(0.4, 0.0),
                lifecycle=ObjectLifecycle.HELD,
            ),
        },
        revision=3,
    )
    T_scene_camera = _pose(1.0, 2.0, 0.0)
    adapter = RRTrackSceneAdapter(T_scene_camera)
    update = adapter.update(
        scene,
        [
            RRTrackInstanceSample("t:0", "t", _output(_pose(0.1, 0.2)), 5.0),
            RRTrackInstanceSample(
                "panel:0", "panel", _output(_pose(0.2, 0.3)), 5.0
            ),
        ],
    )

    assert update.updated_instance_ids == ("t:0",)
    assert update.held_instance_ids == ("panel:0",)
    assert update.scene.revision == 4
    assert np.allclose(update.scene.objects["t:0"].pose[:3, 3], [1.1, 2.2, 0.0])
    assert np.allclose(update.scene.objects["panel:0"].pose, scene.objects["panel:0"].pose)
    assert update.scene.objects["panel:0"].lifecycle == ObjectLifecycle.HELD


def test_scene_adapter_does_not_commit_rejected_rrtrack_pose() -> None:
    initial = _pose(0.3, 0.4)
    scene = TaskSceneState({"obj:0": SceneObjectState("obj:0", "obj", initial)})
    adapter = RRTrackSceneAdapter(np.eye(4))
    update = adapter.update(
        scene,
        [RRTrackInstanceSample("obj:0", "obj", _output(_pose(9.0, 9.0), accepted=False), 1.0)],
    )

    assert update.updated_instance_ids == ()
    assert update.rejected_instance_ids == ("obj:0",)
    assert update.scene.revision == scene.revision
    assert np.allclose(update.scene.objects["obj:0"].pose, initial)
    assert update.scene.objects["obj:0"].metadata["rrtrack"]["accepted"] is False


def test_pusht_tracker_projects_same_rrtrack_pose_into_table_frame() -> None:
    samples = iter(
        [
            RRTrackInstanceSample("t:0", "t", _output(_pose(0.1, 0.0, yaw=0.0), frame_index=1), 1.0),
            RRTrackInstanceSample("t:0", "t", _output(_pose(0.2, 0.0, yaw=0.1), frame_index=2), 2.0),
        ]
    )
    T_table_camera = np.eye(4)
    T_table_camera[1, 3] = 0.3
    tracker = RRTrackPushTTracker(
        lambda: next(samples),
        T_table_camera=T_table_camera,
    )

    first = tracker.observe()
    second = tracker.observe()

    assert np.allclose(first.state.pose.xy, [0.1, 0.3])
    assert np.allclose(second.state.pose.xy, [0.2, 0.3])
    assert np.allclose(second.state.linear_velocity_xy, [0.1, 0.0])
    assert np.isclose(second.state.angular_velocity, 0.1)
    assert second.metadata["source"] == "pose_matrix_push_t_tracker"
    assert second.metadata["rrtrack_accepted"] is True


def test_pusht_tracker_reuses_rrtrack_acceptance_as_confidence_gate() -> None:
    sample = RRTrackInstanceSample(
        "t:0",
        "t",
        _output(_pose(0.0, 0.0), accepted=False),
        1.0,
    )
    tracker = RRTrackPushTTracker(lambda: sample, T_table_camera=np.eye(4))
    observation = tracker.observe()
    assert observation.confidence == 0.0
    assert observation.metadata["rrtrack_precision"] == 0.9
