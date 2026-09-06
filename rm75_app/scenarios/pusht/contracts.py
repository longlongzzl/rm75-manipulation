"""Contracts for closed-loop Push-T planning and system identification."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np


def wrap_angle(value: float) -> float:
    return float((float(value) + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass(frozen=True)
class PushTPose:
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        values = np.asarray([self.x, self.y, self.yaw], dtype=np.float64)
        if not np.all(np.isfinite(values)):
            raise ValueError("Push-T pose must be finite")
        object.__setattr__(self, "yaw", wrap_angle(self.yaw))

    @property
    def xy(self) -> np.ndarray:
        return np.asarray([self.x, self.y], dtype=np.float64)

    def vector(self) -> np.ndarray:
        return np.asarray([self.x, self.y, self.yaw], dtype=np.float64)


@dataclass(frozen=True)
class PushTState:
    pose: PushTPose
    linear_velocity_xy: Sequence[float] = (0.0, 0.0)
    angular_velocity: float = 0.0

    def __post_init__(self) -> None:
        velocity = np.asarray(self.linear_velocity_xy, dtype=np.float64)
        if velocity.shape != (2,) or not np.all(np.isfinite(velocity)):
            raise ValueError("linear_velocity_xy must be a finite 2-vector")
        if not np.isfinite(self.angular_velocity):
            raise ValueError("angular_velocity must be finite")
        object.__setattr__(self, "linear_velocity_xy", velocity)


@dataclass(frozen=True)
class PushTGoal:
    pose: PushTPose
    position_tolerance_m: float = 0.015
    yaw_tolerance_rad: float = np.deg2rad(8.0)

    def __post_init__(self) -> None:
        if not np.all(np.isfinite([self.position_tolerance_m, self.yaw_tolerance_rad])):
            raise ValueError("Push-T goal tolerances must be finite")
        if self.position_tolerance_m <= 0.0 or self.yaw_tolerance_rad <= 0.0:
            raise ValueError("Push-T goal tolerances must be positive")

    def reached(self, state: PushTState) -> bool:
        position_error = float(np.linalg.norm(state.pose.xy - self.pose.xy))
        yaw_error = abs(wrap_angle(state.pose.yaw - self.pose.yaw))
        return (
            position_error <= self.position_tolerance_m
            and yaw_error <= self.yaw_tolerance_rad
        )


@dataclass(frozen=True)
class PushAction:
    """One short quasi-static push.

    ``contact_local_xy`` is in the T-object body frame; direction is in the
    world/table frame.
    """

    contact_local_xy: Sequence[float]
    direction_world_xy: Sequence[float]
    distance_m: float
    speed_mps: float = 0.04
    approach_clearance_m: float = 0.02

    def __post_init__(self) -> None:
        contact = np.asarray(self.contact_local_xy, dtype=np.float64)
        direction = np.asarray(self.direction_world_xy, dtype=np.float64)
        if contact.shape != (2,) or direction.shape != (2,):
            raise ValueError("Push-T contact and direction must be 2-vectors")
        if not np.all(np.isfinite(contact)) or not np.all(np.isfinite(direction)):
            raise ValueError("Push-T contact and direction must be finite")
        if not np.all(np.isfinite([self.distance_m, self.speed_mps, self.approach_clearance_m])):
            raise ValueError("Push-T distance, speed and clearance must be finite")
        norm = float(np.linalg.norm(direction))
        if norm < 1.0e-9:
            raise ValueError("Push-T direction must not be zero")
        if self.distance_m <= 0.0 or self.speed_mps <= 0.0:
            raise ValueError("Push-T distance and speed must be positive")
        if self.approach_clearance_m < 0.0:
            raise ValueError("Push-T approach clearance must not be negative")
        object.__setattr__(self, "contact_local_xy", contact.copy())
        object.__setattr__(self, "direction_world_xy", direction / norm)


@dataclass(frozen=True)
class PushTModelParameters:
    friction: float = 0.45
    translation_gain: float = 0.82
    rotation_gain: float = 3.0
    linear_damping: float = 0.25
    angular_damping: float = 0.35
    contact_efficiency: float = 0.9
    anisotropy: float = 0.0

    def __post_init__(self) -> None:
        values = np.asarray(
            [
                self.friction,
                self.translation_gain,
                self.rotation_gain,
                self.linear_damping,
                self.angular_damping,
                self.contact_efficiency,
                self.anisotropy,
            ],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("Push-T model parameters must be finite")
        if self.friction < 0.0:
            raise ValueError("friction must not be negative")
        if self.translation_gain <= 0.0 or self.rotation_gain < 0.0:
            raise ValueError("model gains are invalid")
        if not 0.0 <= self.linear_damping <= 1.0 or not 0.0 <= self.angular_damping <= 1.0:
            raise ValueError("damping factors must be in [0, 1]")
        if not 0.0 < self.contact_efficiency <= 1.5:
            raise ValueError("contact_efficiency is outside a reasonable range")


@dataclass(frozen=True)
class PushTObservation:
    state: PushTState
    timestamp_s: float
    confidence: float = 1.0
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not np.isfinite(self.timestamp_s):
            raise ValueError("observation timestamp must be finite")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("observation confidence must be in [0, 1]")
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class PushTPlan:
    action: PushAction
    predicted_states: tuple[PushTState, ...]
    cost: float
    candidate_count: int
    horizon: int
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not np.isfinite(self.cost):
            raise ValueError("Push-T plan cost must be finite")
        if self.candidate_count < 1 or self.horizon < 1:
            raise ValueError("Push-T plan counts must be positive")
        object.__setattr__(self, "predicted_states", tuple(self.predicted_states))
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


@dataclass(frozen=True)
class PushTTransition:
    before: PushTObservation
    action: PushAction
    after: PushTObservation
    plan: PushTPlan
    parameter_estimate: PushTModelParameters
    execution_diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "execution_diagnostics",
            dict(self.execution_diagnostics),
        )


@dataclass(frozen=True)
class PushTRunReport:
    success: bool
    goal: PushTGoal
    transitions: tuple[PushTTransition, ...]
    final_observation: PushTObservation
    reason: str
    parameter_estimate: PushTModelParameters
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "transitions", tuple(self.transitions))
        object.__setattr__(self, "metadata", dict(self.metadata))
