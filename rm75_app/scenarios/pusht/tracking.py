"""Adapters from 6D object tracking to planar Push-T observations."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol, Sequence

import numpy as np

from .contracts import PushTObservation, PushTPose, PushTState, wrap_angle


def _transform(value: Sequence[Sequence[float]], name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 transform")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError(f"{name} is not homogeneous")
    return matrix.copy()


@dataclass(frozen=True)
class PoseMatrixSample:
    """One source-frame 6D pose produced by an existing tracker."""

    T_source_object: Sequence[Sequence[float]]
    timestamp_s: float
    confidence: float = 1.0
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "T_source_object",
            _transform(self.T_source_object, "T_source_object"),
        )
        if not np.isfinite(self.timestamp_s):
            raise ValueError("tracker timestamp must be finite")
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise ValueError("tracker confidence must be in [0, 1]")
        object.__setattr__(self, "metadata", dict(self.metadata))


class PoseMatrixProvider(Protocol):
    def __call__(self) -> PoseMatrixSample: ...


class PoseMatrixPushTTracker:
    """Project an existing 6D pose stream into the table plane.

    `T_table_source` maps the source tracker frame (camera/world/base) into the
    table planning frame.  The object's local +X axis defines zero yaw after an
    optional calibrated `object_yaw_offset_rad`.
    """

    def __init__(
        self,
        provider: PoseMatrixProvider | Callable[[], PoseMatrixSample],
        *,
        T_table_source: Sequence[Sequence[float]] | None = None,
        object_yaw_offset_rad: float = 0.0,
        smoothing_alpha: float = 1.0,
        strict_monotonic_timestamps: bool = True,
        minimum_dt_s: float = 1.0e-4,
    ) -> None:
        self.provider = provider
        self.T_table_source = (
            np.eye(4, dtype=np.float64)
            if T_table_source is None
            else _transform(T_table_source, "T_table_source")
        )
        self.object_yaw_offset_rad = float(object_yaw_offset_rad)
        self.smoothing_alpha = float(smoothing_alpha)
        self.strict_monotonic_timestamps = bool(strict_monotonic_timestamps)
        self.minimum_dt_s = float(minimum_dt_s)
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if self.minimum_dt_s <= 0.0:
            raise ValueError("minimum_dt_s must be positive")
        self._previous: PushTObservation | None = None

    def reset(self) -> None:
        self._previous = None

    def _planar_pose(self, sample: PoseMatrixSample) -> PushTPose:
        transform = self.T_table_source @ np.asarray(
            sample.T_source_object,
            dtype=np.float64,
        )
        raw_yaw = float(
            np.arctan2(transform[1, 0], transform[0, 0])
            + self.object_yaw_offset_rad
        )
        raw_xy = transform[:2, 3].astype(np.float64)
        if self._previous is None or self.smoothing_alpha >= 1.0:
            return PushTPose(
                float(raw_xy[0]),
                float(raw_xy[1]),
                raw_yaw,
            )
        previous = self._previous.state.pose
        alpha = self.smoothing_alpha
        filtered_xy = (
            (1.0 - alpha) * previous.xy + alpha * raw_xy
        )
        filtered_yaw = wrap_angle(
            previous.yaw + alpha * wrap_angle(raw_yaw - previous.yaw)
        )
        return PushTPose(
            float(filtered_xy[0]),
            float(filtered_xy[1]),
            filtered_yaw,
        )

    def observe(self) -> PushTObservation:
        sample = self.provider()
        if not isinstance(sample, PoseMatrixSample):
            raise TypeError("pose provider must return PoseMatrixSample")
        previous = self._previous
        if previous is not None:
            dt = float(sample.timestamp_s - previous.timestamp_s)
            if self.strict_monotonic_timestamps and dt <= 0.0:
                raise ValueError(
                    "stale/non-monotonic Push-T tracker timestamp: "
                    f"{sample.timestamp_s} <= {previous.timestamp_s}"
                )
        pose = self._planar_pose(sample)
        velocity = np.zeros(2, dtype=np.float64)
        angular_velocity = 0.0
        if previous is not None:
            dt = float(sample.timestamp_s - previous.timestamp_s)
            if dt >= self.minimum_dt_s:
                velocity = (pose.xy - previous.state.pose.xy) / dt
                angular_velocity = (
                    wrap_angle(pose.yaw - previous.state.pose.yaw) / dt
                )
        observation = PushTObservation(
            PushTState(pose, velocity, angular_velocity),
            float(sample.timestamp_s),
            float(sample.confidence),
            metadata={
                **dict(sample.metadata),
                "source": "pose_matrix_push_t_tracker",
                "T_table_source": self.T_table_source.tolist(),
                "object_yaw_offset_rad": self.object_yaw_offset_rad,
                "smoothing_alpha": self.smoothing_alpha,
            },
        )
        self._previous = observation
        return observation
