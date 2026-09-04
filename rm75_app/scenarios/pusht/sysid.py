"""Online selection/calibration of Push-T simulation parameters."""

from __future__ import annotations

from dataclasses import asdict
from itertools import product
from typing import Iterable

import numpy as np

from .contracts import (
    PushAction,
    PushTModelParameters,
    PushTObservation,
    PushTState,
)
from .model import QuasiStaticPushTModel


class PushTParameterEnsemble:
    """Bayesian-style discrete system identification over simulator settings."""

    def __init__(
        self,
        hypotheses: Iterable[PushTModelParameters],
        *,
        observation_sigma_m: float = 0.012,
        minimum_weight: float = 1.0e-8,
    ) -> None:
        self.hypotheses = tuple(hypotheses)
        if not self.hypotheses:
            raise ValueError("parameter ensemble must contain at least one hypothesis")
        self.observation_sigma_m = float(observation_sigma_m)
        self.minimum_weight = float(minimum_weight)
        if self.observation_sigma_m <= 0.0:
            raise ValueError("observation_sigma_m must be positive")
        self.weights = np.full(
            len(self.hypotheses),
            1.0 / len(self.hypotheses),
            dtype=np.float64,
        )
        self.last_errors = np.full(
            len(self.hypotheses),
            np.nan,
            dtype=np.float64,
        )
        self.update_count = 0

    @classmethod
    def default_grid(cls) -> "PushTParameterEnsemble":
        """Moderate grid suitable for online one-step matching."""

        hypotheses = []
        for friction, translation, rotation, efficiency, anisotropy in product(
            (0.25, 0.45, 0.70),
            (0.68, 0.82, 0.96),
            (2.0, 3.0, 4.2),
            (0.78, 0.92),
            (-0.12, 0.0, 0.12),
        ):
            hypotheses.append(
                PushTModelParameters(
                    friction=friction,
                    translation_gain=translation,
                    rotation_gain=rotation,
                    contact_efficiency=efficiency,
                    anisotropy=anisotropy,
                )
            )
        return cls(hypotheses)

    def reset(self) -> None:
        self.weights.fill(1.0 / len(self.weights))
        self.last_errors.fill(np.nan)
        self.update_count = 0

    def best(self) -> PushTModelParameters:
        return self.hypotheses[int(np.argmax(self.weights))]

    def estimate(self) -> PushTModelParameters:
        fields = tuple(asdict(self.hypotheses[0]))
        values = {
            key: float(
                np.sum(
                    self.weights
                    * np.asarray(
                        [getattr(item, key) for item in self.hypotheses],
                        dtype=np.float64,
                    )
                )
            )
            for key in fields
        }
        return PushTModelParameters(**values)

    @property
    def effective_sample_size(self) -> float:
        return float(1.0 / np.sum(self.weights * self.weights))

    def update(
        self,
        before: PushTObservation | PushTState,
        action: PushAction,
        after: PushTObservation | PushTState,
        model: QuasiStaticPushTModel,
    ) -> dict[str, object]:
        before_state = before.state if isinstance(before, PushTObservation) else before
        after_state = after.state if isinstance(after, PushTObservation) else after
        confidence = (
            min(before.confidence, after.confidence)
            if isinstance(before, PushTObservation)
            and isinstance(after, PushTObservation)
            else 1.0
        )
        predictions = [
            model.step(before_state, action, hypothesis)
            for hypothesis in self.hypotheses
        ]
        errors = np.asarray(
            [
                model.pose_error(predicted, after_state)
                for predicted in predictions
            ],
            dtype=np.float64,
        )
        sigma = self.observation_sigma_m / max(float(confidence), 0.15)
        log_likelihood = -0.5 * np.square(errors / sigma)
        log_likelihood -= float(np.max(log_likelihood))
        likelihood = np.exp(log_likelihood)
        posterior = self.weights * likelihood
        posterior = np.maximum(posterior, self.minimum_weight)
        posterior_sum = float(np.sum(posterior))
        if not np.isfinite(posterior_sum) or posterior_sum <= 0.0:
            posterior.fill(1.0 / len(posterior))
        else:
            posterior /= posterior_sum
        self.weights = posterior
        self.last_errors = errors
        self.update_count += 1
        best_index = int(np.argmax(self.weights))
        return {
            "update_count": self.update_count,
            "observation_confidence": float(confidence),
            "sigma_m": sigma,
            "best_index": best_index,
            "best_weight": float(self.weights[best_index]),
            "best_error_m": float(errors[best_index]),
            "weighted_error_m": float(np.sum(self.weights * errors)),
            "effective_sample_size": self.effective_sample_size,
            "best_parameters": asdict(self.hypotheses[best_index]),
        }

    def diagnostics(self, *, top_k: int = 5) -> dict[str, object]:
        order = np.argsort(-self.weights)[: max(1, int(top_k))]
        return {
            "hypothesis_count": len(self.hypotheses),
            "update_count": self.update_count,
            "effective_sample_size": self.effective_sample_size,
            "weighted_estimate": asdict(self.estimate()),
            "top": [
                {
                    "index": int(index),
                    "weight": float(self.weights[index]),
                    "last_error_m": (
                        None
                        if not np.isfinite(self.last_errors[index])
                        else float(self.last_errors[index])
                    ),
                    "parameters": asdict(self.hypotheses[index]),
                }
                for index in order
            ],
        }
