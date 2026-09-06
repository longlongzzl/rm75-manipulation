"""Random-shooting model-predictive control for Push-T."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .contracts import (
    PushAction,
    PushTGoal,
    PushTModelParameters,
    PushTPlan,
    PushTState,
    wrap_angle,
)
from .model import QuasiStaticPushTModel


@dataclass(frozen=True)
class PushTMPCConfig:
    horizon: int = 3
    candidate_sequences: int = 384
    minimum_push_m: float = 0.012
    maximum_push_m: float = 0.045
    push_speed_mps: float = 0.04
    direction_noise_rad: float = np.deg2rad(35.0)
    goal_steering_fraction: float = 0.55
    position_weight: float = 1.0
    yaw_weight: float = 0.08
    intermediate_weight: float = 0.20
    effort_weight: float = 0.025
    regress_penalty: float = 2.0
    seed: int = 0

    def __post_init__(self) -> None:
        if any(isinstance(value, bool) or not isinstance(value, int) or value < 1
               for value in (self.horizon, self.candidate_sequences)):
            raise ValueError("MPC horizon and candidate count must be positive")
        if not all(np.isfinite(value) for value in (
            self.minimum_push_m, self.maximum_push_m, self.push_speed_mps,
            self.direction_noise_rad, self.goal_steering_fraction, self.position_weight,
            self.yaw_weight, self.intermediate_weight, self.effort_weight, self.regress_penalty,
        )):
            raise ValueError("MPC numeric parameters must be finite")
        if self.push_speed_mps <= 0 or any(value < 0 for value in (
            self.direction_noise_rad, self.position_weight, self.yaw_weight,
            self.intermediate_weight, self.effort_weight, self.regress_penalty,
        )):
            raise ValueError("MPC speed must be positive and costs nonnegative")
        if not 0.0 < self.minimum_push_m <= self.maximum_push_m:
            raise ValueError("invalid Push-T push distance range")
        if not 0.0 <= self.goal_steering_fraction <= 1.0:
            raise ValueError("goal_steering_fraction must be in [0, 1]")


class PushTMPC:
    """Simulate many short futures and execute only the first push."""

    def __init__(
        self,
        model: QuasiStaticPushTModel,
        config: PushTMPCConfig | None = None,
    ) -> None:
        self.model = model
        self.config = config or PushTMPCConfig()
        self._rng = np.random.default_rng(self.config.seed)

    @staticmethod
    def _rotate(vector: np.ndarray, angle: float) -> np.ndarray:
        c, s = float(np.cos(angle)), float(np.sin(angle))
        return np.asarray(
            [c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]],
            dtype=np.float64,
        )

    def _sample_action(
        self,
        state: PushTState,
        goal: PushTGoal,
        contact_local: np.ndarray,
    ) -> PushAction:
        rotation = self.model._rotation(state.pose.yaw)
        contact_world = rotation @ contact_local
        inward = -contact_world
        inward_norm = float(np.linalg.norm(inward))
        if inward_norm < 1.0e-9:
            inward = np.asarray([1.0, 0.0], dtype=np.float64)
        else:
            inward /= inward_norm
        goal_direction = goal.pose.xy - state.pose.xy
        goal_norm = float(np.linalg.norm(goal_direction))
        if goal_norm < 1.0e-9:
            goal_direction = inward
        else:
            goal_direction /= goal_norm
        steering = self.config.goal_steering_fraction
        direction = (1.0 - steering) * inward + steering * goal_direction
        if float(np.linalg.norm(direction)) < 1.0e-9:
            direction = inward
        direction /= float(np.linalg.norm(direction))
        direction = self._rotate(
            direction,
            float(
                self._rng.uniform(
                    -self.config.direction_noise_rad,
                    self.config.direction_noise_rad,
                )
            ),
        )
        if float(np.dot(direction, inward)) < 0.10:
            direction = direction + inward * (
                0.10 - float(np.dot(direction, inward)) + 1.0e-6
            )
            direction /= float(np.linalg.norm(direction))
        distance = float(
            self._rng.uniform(
                self.config.minimum_push_m,
                self.config.maximum_push_m,
            )
        )
        return PushAction(
            contact_local,
            direction,
            distance,
            self.config.push_speed_mps,
        )

    def _sequence(
        self,
        state: PushTState,
        goal: PushTGoal,
        parameters: PushTModelParameters,
        contacts: Sequence[np.ndarray],
    ) -> tuple[tuple[PushAction, ...], tuple[PushTState, ...]]:
        actions: list[PushAction] = []
        states = [state]
        current = state
        for _ in range(self.config.horizon):
            contact = contacts[int(self._rng.integers(0, len(contacts)))]
            action = self._sample_action(current, goal, contact)
            actions.append(action)
            current = self.model.step(current, action, parameters)
            states.append(current)
        return tuple(actions), tuple(states)

    def _cost(
        self,
        initial: PushTState,
        states: tuple[PushTState, ...],
        actions: tuple[PushAction, ...],
        goal: PushTGoal,
    ) -> float:
        final = states[-1]
        position_error = float(np.linalg.norm(final.pose.xy - goal.pose.xy))
        yaw_error = abs(wrap_angle(final.pose.yaw - goal.pose.yaw))
        cost = (
            self.config.position_weight * position_error * position_error
            + self.config.yaw_weight * yaw_error * yaw_error
        )
        for state in states[1:-1]:
            intermediate_position = float(
                np.linalg.norm(state.pose.xy - goal.pose.xy)
            )
            intermediate_yaw = abs(
                wrap_angle(state.pose.yaw - goal.pose.yaw)
            )
            cost += self.config.intermediate_weight * (
                self.config.position_weight
                * intermediate_position
                * intermediate_position
                + self.config.yaw_weight
                * intermediate_yaw
                * intermediate_yaw
            )
        cost += self.config.effort_weight * sum(
            float(item.distance_m) for item in actions
        )
        initial_error = float(
            np.linalg.norm(initial.pose.xy - goal.pose.xy)
        )
        if position_error > initial_error + 1.0e-6:
            cost += self.config.regress_penalty * (
                position_error - initial_error
            )
        return float(cost)

    def plan(
        self,
        state: PushTState,
        goal: PushTGoal,
        parameters: PushTModelParameters,
    ) -> PushTPlan:
        state_validator = getattr(self.model, "state_is_valid", None)
        if callable(state_validator) and not state_validator(state):
            raise ValueError("initial Push-T state is outside the model workspace")
        contacts = self.model.geometry.candidate_contact_points()
        best_actions: tuple[PushAction, ...] | None = None
        best_states: tuple[PushTState, ...] | None = None
        best_cost = float("inf")
        costs = np.empty(self.config.candidate_sequences, dtype=np.float64)
        for index in range(self.config.candidate_sequences):
            actions, states = self._sequence(
                state,
                goal,
                parameters,
                contacts,
            )
            valid = not callable(state_validator) or all(state_validator(item) for item in states)
            cost = self._cost(state, states, actions, goal) if valid else float("inf")
            costs[index] = cost
            if cost < best_cost:
                best_cost = cost
                best_actions = actions
                best_states = states
        if best_actions is None or best_states is None:
            raise RuntimeError("Push-T MPC did not generate any candidate")
        return PushTPlan(
            best_actions[0],
            best_states,
            best_cost,
            self.config.candidate_sequences,
            self.config.horizon,
            diagnostics={
                "planner_algorithm": "random_shooting_mpc",
                "invalid_future_count": int(np.sum(~np.isfinite(costs))),
                "model_step_evaluations": self.config.candidate_sequences * self.config.horizon,
                "cost_p10": float(np.percentile(costs[np.isfinite(costs)], 10.0)),
                "cost_median": float(np.median(costs[np.isfinite(costs)])),
                "cost_p90": float(np.percentile(costs[np.isfinite(costs)], 90.0)),
                "candidate_contact_count": len(contacts),
                "future_actions": [
                    {
                        "contact_local_xy": item.contact_local_xy.tolist(),
                        "direction_world_xy": item.direction_world_xy.tolist(),
                        "distance_m": item.distance_m,
                    }
                    for item in best_actions
                ],
            },
        )
