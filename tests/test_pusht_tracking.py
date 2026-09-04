from __future__ import annotations

import numpy as np

from rm75_app.scenarios.pusht import (
    PoseMatrixPushTTracker,
    PoseMatrixSample,
    wrap_angle,
)


def _pose(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]
    value[:3, 3] = [x, y, 0.0]
    return value


def test_pose_matrix_tracker_transforms_frame_and_estimates_velocity() -> None:
    samples = iter(
        (
            PoseMatrixSample(_pose(0.1, 0.0, 0.0), 1.0, 0.9),
            PoseMatrixSample(_pose(0.2, 0.0, 0.1), 2.0, 0.8),
        )
    )
    T_table_source = np.eye(4)
    T_table_source[1, 3] = 0.3
    tracker = PoseMatrixPushTTracker(
        lambda: next(samples),
        T_table_source=T_table_source,
    )

    first = tracker.observe()
    second = tracker.observe()

    assert np.allclose(first.state.pose.xy, [0.1, 0.3])
    assert np.allclose(second.state.pose.xy, [0.2, 0.3])
    assert np.allclose(second.state.linear_velocity_xy, [0.1, 0.0])
    assert np.isclose(second.state.angular_velocity, 0.1)
    assert second.confidence == 0.8


def test_pose_matrix_tracker_smooths_yaw_across_pi_without_large_jump() -> None:
    samples = iter(
        (
            PoseMatrixSample(_pose(0.0, 0.0, np.deg2rad(179.0)), 1.0),
            PoseMatrixSample(_pose(0.0, 0.0, np.deg2rad(-179.0)), 2.0),
        )
    )
    tracker = PoseMatrixPushTTracker(
        lambda: next(samples),
        smoothing_alpha=0.5,
    )

    first = tracker.observe()
    second = tracker.observe()

    circular_delta = wrap_angle(
        second.state.pose.yaw - first.state.pose.yaw
    )
    assert abs(circular_delta) <= np.deg2rad(1.01)
    assert abs(second.state.angular_velocity) <= np.deg2rad(1.01)


def test_pose_matrix_tracker_rejects_stale_timestamp() -> None:
    samples = iter(
        (
            PoseMatrixSample(_pose(0.0, 0.0, 0.0), 2.0),
            PoseMatrixSample(_pose(0.0, 0.0, 0.0), 2.0),
        )
    )
    tracker = PoseMatrixPushTTracker(lambda: next(samples))
    tracker.observe()

    try:
        tracker.observe()
    except ValueError as exc:
        assert "timestamp" in str(exc)
    else:
        raise AssertionError("stale tracker timestamp was accepted")
