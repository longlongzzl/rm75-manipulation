"""Closed-loop Push-T observe -> imagine -> push -> track -> replan control."""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from typing import Any, Callable, Mapping, Protocol

import numpy as np

from .contracts import (
    PushAction,
    PushTGoal,
    PushTModelParameters,
    PushTObservation,
    PushTPose,
    PushTRunReport,
    PushTState,
    PushTTransition,
)
from .model import QuasiStaticPushTModel
from .mpc import PushTMPC
from .sysid import PushTParameterEnsemble


class PushTTracker(Protocol):
    def observe(self) -> PushTObservation: ...


class PushTExecutor(Protocol):
    def execute_push(
        self,
        action: PushAction,
        observation: PushTObservation,
    ) -> Mapping[str, Any]: ...


class CartesianPushBackend(Protocol):
    """Physical/simulation backend for one pusher contact motion."""

    def execute_cartesian_push(
        self,
        *,
        approach_xy: np.ndarray,
        contact_xy: np.ndarray,
        end_xy: np.ndarray,
        speed_mps: float,
        metadata: Mapping[str, Any],
    ) -> Mapping[str, Any]: ...


@dataclass(frozen=True)
class PushTControllerConfig:
    max_steps: int = 20
    minimum_tracking_confidence: float = 0.45
    settle_time_s: float = 0.15
    minimum_progress_m: float = 0.001
    max_consecutive_stalls: int = 3
    max_observation_jump_m: float = 0.20

    def __post_init__(self) -> None:
        if self.max_steps < 1 or self.max_consecutive_stalls < 1:
            raise ValueError("Push-T controller step limits must be positive")
        if not 0.0 <= self.minimum_tracking_confidence <= 1.0:
            raise ValueError("tracking confidence threshold must be in [0, 1]")
        if self.settle_time_s < 0.0:
            raise ValueError("settle time must not be negative")


class ObjectFramePushExecutor:
    """Convert an object-frame PushAction into a world-table push segment."""

    def __init__(self, backend: CartesianPushBackend) -> None:
        self.backend = backend

    @staticmethod
    def _rotation(yaw: float) -> np.ndarray:
        c, s = float(np.cos(yaw)), float(np.sin(yaw))
        return np.asarray([[c, -s], [s, c]], dtype=np.float64)

    def execute_push(
        self,
        action: PushAction,
        observation: PushTObservation,
    ) -> Mapping[str, Any]:
        rotation = self._rotation(observation.state.pose.yaw)
        contact_xy = (
            observation.state.pose.xy
            + rotation @ np.asarray(action.contact_local_xy, dtype=np.float64)
        )
        direction = np.asarray(action.direction_world_xy, dtype=np.float64)
        approach_xy = contact_xy - direction * action.approach_clearance_m
        end_xy = contact_xy + direction * action.distance_m
        result = self.backend.execute_cartesian_push(
            approach_xy=approach_xy,
            contact_xy=contact_xy,
            end_xy=end_xy,
            speed_mps=action.speed_mps,
            metadata={
                "contact_local_xy": action.contact_local_xy.tolist(),
                "observation_timestamp_s": observation.timestamp_s,
            },
        )
        return {
            "approach_xy": approach_xy.tolist(),
            "contact_xy": contact_xy.tolist(),
            "end_xy": end_xy.tolist(),
            **dict(result),
        }


class PushTClosedLoopController:
    """Execute the first action of each simulated future and re-observe."""

    def __init__(
        self,
        tracker: PushTTracker,
        executor: PushTExecutor,
        model: QuasiStaticPushTModel,
        mpc: PushTMPC,
        *,
        parameters: PushTModelParameters | None = None,
        estimator: PushTParameterEnsemble | None = None,
        config: PushTControllerConfig | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.tracker = tracker
        self.executor = executor
        self.model = model
        self.mpc = mpc
        self.nominal_parameters = parameters or PushTModelParameters()
        self.estimator = estimator
        self.config = config or PushTControllerConfig()
        self.sleep = sleep

    def _parameters(self) -> PushTModelParameters:
        return (
            self.nominal_parameters
            if self.estimator is None
            else self.estimator.estimate()
        )

    def run(self, goal: PushTGoal) -> PushTRunReport:
        transitions: list[PushTTransition] = []
        stalls = 0
        observation = self.tracker.observe()
        if observation.confidence < self.config.minimum_tracking_confidence:
            return PushTRunReport(
                False,
                goal,
                (),
                observation,
                "initial_tracking_confidence_too_low",
                self._parameters(),
            )
        if goal.reached(observation.state):
            return PushTRunReport(
                True,
                goal,
                (),
                observation,
                "already_at_goal",
                self._parameters(),
            )

        for step_index in range(self.config.max_steps):
            parameters = self._parameters()
            plan = self.mpc.plan(observation.state, goal, parameters)
            execution = dict(
                self.executor.execute_push(plan.action, observation)
            )
            if self.config.settle_time_s > 0.0:
                self.sleep(self.config.settle_time_s)
            after = self.tracker.observe()
            if after.confidence < self.config.minimum_tracking_confidence:
                return PushTRunReport(
                    False,
                    goal,
                    tuple(transitions),
                    after,
                    "tracking_confidence_too_low",
                    self._parameters(),
                    metadata={"failed_step": step_index},
                )
            jump = float(
                np.linalg.norm(
                    after.state.pose.xy - observation.state.pose.xy
                )
            )
            if jump > self.config.max_observation_jump_m:
                return PushTRunReport(
                    False,
                    goal,
                    tuple(transitions),
                    after,
                    "tracking_jump_rejected",
                    self._parameters(),
                    metadata={
                        "failed_step": step_index,
                        "observed_jump_m": jump,
                    },
                )

            before_error = float(
                np.linalg.norm(observation.state.pose.xy - goal.pose.xy)
            )
            after_error = float(
                np.linalg.norm(after.state.pose.xy - goal.pose.xy)
            )
            progress = before_error - after_error
            if progress < self.config.minimum_progress_m:
                stalls += 1
            else:
                stalls = 0

            system_identification: Mapping[str, Any] = {}
            if self.estimator is not None:
                system_identification = self.estimator.update(
                    observation,
                    plan.action,
                    after,
                    self.model,
                )
            execution.update(
                {
                    "step_index": step_index,
                    "position_error_before_m": before_error,
                    "position_error_after_m": after_error,
                    "progress_m": progress,
                    "consecutive_stalls": stalls,
                    "system_identification": dict(system_identification),
                }
            )
            transitions.append(
                PushTTransition(
                    observation,
                    plan.action,
                    after,
                    plan,
                    parameters,
                    execution,
                )
            )
            observation = after
            if goal.reached(observation.state):
                return PushTRunReport(
                    True,
                    goal,
                    tuple(transitions),
                    observation,
                    "goal_reached",
                    self._parameters(),
                    metadata={
                        "parameter_estimator": (
                            None
                            if self.estimator is None
                            else self.estimator.diagnostics()
                        )
                    },
                )
            if stalls >= self.config.max_consecutive_stalls:
                return PushTRunReport(
                    False,
                    goal,
                    tuple(transitions),
                    observation,
                    "stalled",
                    self._parameters(),
                    metadata={
                        "parameter_estimator": (
                            None
                            if self.estimator is None
                            else self.estimator.diagnostics()
                        )
                    },
                )
        return PushTRunReport(
            False,
            goal,
            tuple(transitions),
            observation,
            "step_limit",
            self._parameters(),
            metadata={
                "parameter_estimator": (
                    None
                    if self.estimator is None
                    else self.estimator.diagnostics()
                )
            },
        )


class SimulatedPushTWorld:
    """Pure-Python tracker/executor used for dry runs and unit tests."""

    def __init__(
        self,
        model: QuasiStaticPushTModel,
        initial_state: PushTState,
        *,
        true_parameters: PushTModelParameters | None = None,
        observation_noise_std_m: float = 0.0,
        observation_noise_std_rad: float = 0.0,
        seed: int = 0,
    ) -> None:
        self.model = model
        self.state = initial_state
        self.true_parameters = true_parameters or PushTModelParameters()
        self.observation_noise_std_m = float(observation_noise_std_m)
        self.observation_noise_std_rad = float(observation_noise_std_rad)
        self._rng = np.random.default_rng(seed)
        self._time_s = 0.0
        self.actions: list[PushAction] = []

    def observe(self) -> PushTObservation:
        noise_xy = self._rng.normal(
            0.0,
            self.observation_noise_std_m,
            size=2,
        )
        noise_yaw = float(
            self._rng.normal(0.0, self.observation_noise_std_rad)
        )
        pose = PushTPose(
            float(self.state.pose.x + noise_xy[0]),
            float(self.state.pose.y + noise_xy[1]),
            float(self.state.pose.yaw + noise_yaw),
        )
        return PushTObservation(
            PushTState(
                pose,
                self.state.linear_velocity_xy,
                self.state.angular_velocity,
            ),
            self._time_s,
            1.0,
            metadata={"source": "simulated_push_t_world"},
        )

    def execute_push(
        self,
        action: PushAction,
        observation: PushTObservation,
    ) -> Mapping[str, Any]:
        del observation
        before = self.state
        self.state = self.model.step(
            self.state,
            action,
            self.true_parameters,
        )
        self._time_s += float(action.distance_m / action.speed_mps)
        self.actions.append(action)
        return {
            "source": "simulated_push_t_world",
            "true_parameters": asdict(self.true_parameters),
            "true_before": before.pose.vector().tolist(),
            "true_after": self.state.pose.vector().tolist(),
        }
