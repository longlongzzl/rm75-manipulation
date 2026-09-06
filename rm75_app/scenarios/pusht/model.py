"""Lightweight quasi-static Push-T dynamics used by MPC and online fitting."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .contracts import (
    PushAction,
    PushTModelParameters,
    PushTPose,
    PushTState,
    wrap_angle,
)


@dataclass(frozen=True)
class PushTGeometry:
    """Approximate T geometry in metres, centered at its planar COM."""

    crossbar_width_m: float = 0.16
    crossbar_height_m: float = 0.045
    stem_width_m: float = 0.05
    stem_height_m: float = 0.12

    def __post_init__(self) -> None:
        values = (
            self.crossbar_width_m,
            self.crossbar_height_m,
            self.stem_width_m,
            self.stem_height_m,
        )
        if not np.all(np.isfinite(values)) or any(float(item) <= 0.0 for item in values):
            raise ValueError("Push-T geometry dimensions must be positive")

    @property
    def characteristic_radius_m(self) -> float:
        return 0.5 * float(
            np.hypot(
                self.crossbar_width_m,
                self.crossbar_height_m + self.stem_height_m,
            )
        )

    def candidate_contact_points(self) -> tuple[np.ndarray, ...]:
        half_bar = 0.5 * self.crossbar_width_m
        bar_y = -0.5 * self.stem_height_m
        stem_bottom = 0.5 * self.stem_height_m
        half_stem = 0.5 * self.stem_width_m
        return tuple(
            np.asarray(item, dtype=np.float64)
            for item in (
                (-half_bar, bar_y),
                (-0.5 * half_bar, bar_y - 0.5 * self.crossbar_height_m),
                (0.0, bar_y - 0.5 * self.crossbar_height_m),
                (0.5 * half_bar, bar_y - 0.5 * self.crossbar_height_m),
                (half_bar, bar_y),
                (-half_stem, stem_bottom),
                (0.0, stem_bottom),
                (half_stem, stem_bottom),
                (-half_stem, 0.0),
                (half_stem, 0.0),
            )
        )


class QuasiStaticPushTModel:
    """Fast planar model for many-future search and one-step calibration.

    This is not presented as a high-fidelity contact simulator. The parameter
    ensemble is deliberately exposed so real transitions can select the
    simulated response that best matches the physical T object.
    """

    def __init__(
        self,
        geometry: PushTGeometry | None = None,
        *,
        workspace_bounds_xy: tuple[float, float, float, float] | None = None,
    ) -> None:
        self.geometry = geometry or PushTGeometry()
        if workspace_bounds_xy is not None:
            if len(workspace_bounds_xy) != 4 or not np.all(np.isfinite(workspace_bounds_xy)):
                raise ValueError("workspace bounds must be four finite values")
            xmin, xmax, ymin, ymax = workspace_bounds_xy
            if not xmin < xmax or not ymin < ymax:
                raise ValueError("workspace bounds must have positive area")
        self.workspace_bounds_xy = workspace_bounds_xy

    def state_is_valid(self, state: PushTState) -> bool:
        """Conservative full-object containment; not a collision/safety proof."""
        if self.workspace_bounds_xy is None:
            return True
        xmin, xmax, ymin, ymax = self.workspace_bounds_xy
        radius = self.geometry.characteristic_radius_m
        x, y = state.pose.xy
        return bool(xmin + radius <= x <= xmax - radius
                    and ymin + radius <= y <= ymax - radius)

    @staticmethod
    def _rotation(yaw: float) -> np.ndarray:
        c, s = float(np.cos(yaw)), float(np.sin(yaw))
        return np.asarray([[c, -s], [s, c]], dtype=np.float64)

    def step(
        self,
        state: PushTState,
        action: PushAction,
        parameters: PushTModelParameters,
    ) -> PushTState:
        rotation = self._rotation(state.pose.yaw)
        lever_world = rotation @ np.asarray(
            action.contact_local_xy,
            dtype=np.float64,
        )
        direction = np.asarray(action.direction_world_xy, dtype=np.float64)
        direction_body = rotation.T @ direction

        anisotropic_scale = 1.0 + parameters.anisotropy * (
            abs(float(direction_body[0])) - abs(float(direction_body[1]))
        )
        anisotropic_scale = float(np.clip(anisotropic_scale, 0.25, 1.75))
        friction_scale = 1.0 / (1.0 + parameters.friction)
        displacement = (
            float(action.distance_m)
            * parameters.translation_gain
            * parameters.contact_efficiency
            * friction_scale
            * anisotropic_scale
        )
        delta_xy = direction * displacement

        torque = float(
            lever_world[0] * direction[1]
            - lever_world[1] * direction[0]
        )
        radius = max(self.geometry.characteristic_radius_m, 1.0e-4)
        delta_yaw = (
            parameters.rotation_gain
            * displacement
            * torque
            / (radius * radius)
        )
        delta_yaw = float(np.clip(delta_yaw, -0.45, 0.45))

        new_xy = state.pose.xy + delta_xy
        # Bounds are a planner constraint, not an unmodeled physical wall.
        # Keep the true model prediction and reject it in MPC if out of bounds.

        duration = max(float(action.distance_m / action.speed_mps), 1.0e-3)
        measured_velocity = delta_xy / duration
        previous_velocity = np.asarray(
            state.linear_velocity_xy,
            dtype=np.float64,
        )
        velocity = (
            (1.0 - parameters.linear_damping) * measured_velocity
            + parameters.linear_damping * previous_velocity
        )
        measured_angular = delta_yaw / duration
        angular_velocity = (
            (1.0 - parameters.angular_damping) * measured_angular
            + parameters.angular_damping * float(state.angular_velocity)
        )
        return PushTState(
            PushTPose(
                float(new_xy[0]),
                float(new_xy[1]),
                wrap_angle(state.pose.yaw + delta_yaw),
            ),
            velocity,
            angular_velocity,
        )

    def rollout(
        self,
        state: PushTState,
        actions: Iterable[PushAction],
        parameters: PushTModelParameters,
    ) -> tuple[PushTState, ...]:
        output = [state]
        current = state
        for action in actions:
            current = self.step(current, action, parameters)
            output.append(current)
        return tuple(output)

    @staticmethod
    def pose_error(
        predicted: PushTState,
        observed: PushTState,
        *,
        yaw_scale_m: float = 0.06,
    ) -> float:
        translation = float(
            np.linalg.norm(predicted.pose.xy - observed.pose.xy)
        )
        yaw = abs(wrap_angle(predicted.pose.yaw - observed.pose.yaw))
        return float(np.hypot(translation, yaw_scale_m * yaw))
