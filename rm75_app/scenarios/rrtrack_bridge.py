"""Shared bridge from the PickPlace RRTrack chain into all three scenarios.

This module deliberately does *not* implement a second tracker. Sorting,
magnetic assembly and Push-T all consume the same RRTrack result produced by
``rm75_app.perception.rrtrack``. Scenario-specific code only changes how that
6D pose is consumed:

* sorting / magnetic assembly update the planning ``TaskSceneState``;
* Push-T projects the same 6D pose into table-frame ``(x, y, yaw)`` through the
  existing ``PoseMatrixPushTTracker`` adapter.

The bridge uses a structural protocol instead of importing the full RRTrack
package. This keeps the lightweight scenario API free from OpenCV / tracker
runtime dependencies while remaining directly compatible with ``RRTrackOutput``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable, Mapping, Protocol, Sequence
import math

import numpy as np

from rm75_app.orchestration.multi_object_executor import ObjectLifecycle, TaskSceneState

from .pusht.tracking import PoseMatrixPushTTracker, PoseMatrixSample


class RRTrackAgreementLike(Protocol):
    precision: float
    support: float
    entropy: float


class RRTrackOutputLike(Protocol):
    frame_index: int
    state: Any
    T_cam_obj: Any
    agreement: RRTrackAgreementLike
    accepted: bool
    event: str


def _transform(value: Sequence[Sequence[float]], name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 transform")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError(f"{name} must be homogeneous")
    return matrix.copy()


def _state_value(output: RRTrackOutputLike) -> str:
    value = getattr(output.state, "value", output.state)
    return str(value).lower()


def _validate_rrtrack_output(output: RRTrackOutputLike) -> None:
    required = ("frame_index", "state", "T_cam_obj", "agreement", "accepted", "event")
    missing = [name for name in required if not hasattr(output, name)]
    if missing:
        raise TypeError(f"output is not RRTrack-compatible; missing {missing}")
    agreement = output.agreement
    for name in ("precision", "support", "entropy"):
        if not hasattr(agreement, name):
            raise TypeError(f"RRTrack agreement is missing {name!r}")


@dataclass(frozen=True)
class RRTrackInstanceSample:
    """One timestamped result from the existing RRTrack chain."""

    instance_id: str
    asset_name: str
    output: RRTrackOutputLike
    timestamp_s: float

    def __post_init__(self) -> None:
        if not self.instance_id or not self.asset_name:
            raise ValueError("RRTrack sample needs stable instance_id and asset_name")
        _validate_rrtrack_output(self.output)
        if not math.isfinite(float(self.timestamp_s)):
            raise ValueError("RRTrack sample timestamp must be finite")
        object.__setattr__(self, "timestamp_s", float(self.timestamp_s))

    @property
    def pose_accepted(self) -> bool:
        return bool(
            self.output.accepted
            and self.output.T_cam_obj is not None
            and _state_value(self.output) not in {"lost", "recovering"}
        )

    def pose_matrix_sample(self) -> PoseMatrixSample:
        """Expose the exact RRTrack pose to the planar Push-T adapter.

        Confidence intentionally follows RRTrack's own accepted/rejected gate
        (1/0) rather than inventing a second quality threshold. Precision and
        support are retained in metadata for logging and later calibration.
        """

        if self.output.T_cam_obj is None:
            raise ValueError("RRTrack sample has no object pose")
        agreement = self.output.agreement
        return PoseMatrixSample(
            self.output.T_cam_obj,
            self.timestamp_s,
            1.0 if self.pose_accepted else 0.0,
            metadata={
                "source": "rrtrack",
                "instance_id": self.instance_id,
                "asset_name": self.asset_name,
                "rrtrack_state": _state_value(self.output),
                "rrtrack_event": str(self.output.event),
                "rrtrack_accepted": bool(self.output.accepted),
                "rrtrack_precision": float(agreement.precision),
                "rrtrack_support": float(agreement.support),
                "rrtrack_entropy": float(agreement.entropy),
                "rrtrack_frame_index": int(self.output.frame_index),
            },
        )


@dataclass(frozen=True)
class RRTrackSceneUpdate:
    scene: TaskSceneState
    updated_instance_ids: tuple[str, ...]
    rejected_instance_ids: tuple[str, ...]
    missing_instance_ids: tuple[str, ...]
    held_instance_ids: tuple[str, ...]


class RRTrackSceneAdapter:
    """Update a task scene from accepted RRTrack 6D poses.

    The adapter preserves object identity and lifecycle. In particular it does
    not overwrite a ``HELD`` object's planner attachment state by default.
    Rejected/lost tracker outputs are recorded in object metadata but never
    committed as new geometry.
    """

    def __init__(
        self,
        T_scene_camera: Sequence[Sequence[float]],
        *,
        update_held_objects: bool = False,
    ) -> None:
        self.T_scene_camera = _transform(T_scene_camera, "T_scene_camera")
        self.update_held_objects = bool(update_held_objects)

    def update(
        self,
        scene: TaskSceneState,
        samples: Iterable[RRTrackInstanceSample],
    ) -> RRTrackSceneUpdate:
        result = scene.copy()
        updated: list[str] = []
        rejected: list[str] = []
        missing: list[str] = []
        held: list[str] = []
        latest_timestamp: float | None = None

        for sample in samples:
            latest_timestamp = (
                sample.timestamp_s
                if latest_timestamp is None
                else max(latest_timestamp, sample.timestamp_s)
            )
            object_state = result.objects.get(sample.instance_id)
            if object_state is None:
                missing.append(sample.instance_id)
                continue
            if object_state.asset_name != sample.asset_name:
                raise ValueError(
                    f"RRTrack identity mismatch for {sample.instance_id!r}: "
                    f"scene asset={object_state.asset_name!r}, "
                    f"tracker asset={sample.asset_name!r}"
                )

            tracking_metadata = {
                "timestamp_s": sample.timestamp_s,
                "frame_index": int(sample.output.frame_index),
                "state": _state_value(sample.output),
                "event": str(sample.output.event),
                "accepted": bool(sample.output.accepted),
                "precision": float(sample.output.agreement.precision),
                "support": float(sample.output.agreement.support),
            }
            object_state.metadata["rrtrack"] = tracking_metadata

            if (
                object_state.lifecycle == ObjectLifecycle.HELD
                and not self.update_held_objects
            ):
                held.append(sample.instance_id)
                continue
            if not sample.pose_accepted:
                rejected.append(sample.instance_id)
                continue

            T_scene_object = self.T_scene_camera @ np.asarray(
                sample.output.T_cam_obj,
                dtype=np.float64,
            ).reshape(4, 4)
            object_state.pose = _transform(T_scene_object, "T_scene_object")
            updated.append(sample.instance_id)

        if updated:
            result.revision += 1
        if latest_timestamp is not None:
            result.metadata["rrtrack_last_timestamp_s"] = latest_timestamp
        result.metadata["rrtrack_updated_instance_ids"] = list(updated)
        result.metadata["rrtrack_rejected_instance_ids"] = list(rejected)

        return RRTrackSceneUpdate(
            result,
            tuple(updated),
            tuple(rejected),
            tuple(missing),
            tuple(held),
        )


class RRTrackPushTTracker:
    """Thin Push-T view of the existing PickPlace RRTrack stream.

    ``sample_provider`` should return the latest T-object
    ``RRTrackInstanceSample`` from the same tracking service used by PickPlace.
    No segmentation, registration, memory bank or relocalization is duplicated
    here.
    """

    def __init__(
        self,
        sample_provider: Callable[[], RRTrackInstanceSample],
        *,
        T_table_camera: Sequence[Sequence[float]],
        object_yaw_offset_rad: float = 0.0,
        smoothing_alpha: float = 1.0,
        minimum_dt_s: float = 1.0e-4,
    ) -> None:
        self.sample_provider = sample_provider
        self._planar = PoseMatrixPushTTracker(
            self._next_pose,
            T_table_source=T_table_camera,
            object_yaw_offset_rad=object_yaw_offset_rad,
            smoothing_alpha=smoothing_alpha,
            strict_monotonic_timestamps=True,
            minimum_dt_s=minimum_dt_s,
        )

    def _next_pose(self) -> PoseMatrixSample:
        sample = self.sample_provider()
        if not isinstance(sample, RRTrackInstanceSample):
            raise TypeError("sample_provider must return RRTrackInstanceSample")
        return sample.pose_matrix_sample()

    def reset(self) -> None:
        self._planar.reset()

    def observe(self):
        return self._planar.observe()
