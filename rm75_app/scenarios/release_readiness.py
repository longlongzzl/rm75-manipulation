"""Calibration-driven release readiness assessment.

This module deliberately does not invent safe release thresholds.  It converts
measured pre-release motion/contact metrics into a shadow-only diagnostic when
(and only when) the caller supplies calibrated thresholds.  A hard joint-limit
excursion is always treated as not-ready.

The assessor is backend-neutral so the same contract can be used for ManiSkill
instrumentation first and, after independent calibration, a real-robot observer.
It never commands a gripper by itself.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Mapping
import math


@dataclass(frozen=True)
class ReleaseReadinessThresholds:
    """Experimentally calibrated upper bounds for a release window.

    Values are intentionally mandatory.  There are no production defaults:
    callers must derive these bounds from labelled release data rather than
    silently inheriting simulator-specific numbers.
    """

    max_object_linear_speed_m_s: float
    max_object_angular_speed_rad_s: float
    max_tcp_linear_speed_m_s: float
    max_tcp_angular_speed_rad_s: float
    max_pad_origin_distance_rate_m_s: float
    max_arm_tracking_error_rad: float
    max_gripper_object_penetration_m: float
    max_object_lateral_impulse_norm_sum_ns: float
    max_support_relative_translation_error_m: float

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            value = float(value)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be a finite non-negative value")
            object.__setattr__(self, name, value)

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "ReleaseReadinessThresholds":
        expected = set(cls.__dataclass_fields__)
        missing = sorted(expected - set(value))
        extra = sorted(set(value) - expected)
        if missing or extra:
            raise ValueError(
                "release threshold schema mismatch: "
                f"missing={missing}, extra={extra}"
            )
        return cls(**{key: float(value[key]) for key in expected})

    def as_dict(self) -> dict[str, float]:
        return {key: float(value) for key, value in asdict(self).items()}


@dataclass(frozen=True)
class ReleaseReadinessAssessment:
    state: str
    reasons: tuple[str, ...]
    observed: Mapping[str, float]
    thresholds: Mapping[str, float] | None
    shadow_only: bool = True

    def __post_init__(self) -> None:
        if self.state not in {"ready", "not-ready", "unknown"}:
            raise ValueError(f"unsupported readiness state {self.state!r}")
        object.__setattr__(self, "reasons", tuple(self.reasons))
        object.__setattr__(
            self,
            "observed",
            {str(key): float(value) for key, value in self.observed.items()},
        )
        if self.thresholds is not None:
            object.__setattr__(
                self,
                "thresholds",
                {str(key): float(value) for key, value in self.thresholds.items()},
            )

    def as_dict(self) -> dict[str, object]:
        return {
            "state": self.state,
            "reasons": list(self.reasons),
            "observed": dict(self.observed),
            "thresholds": None if self.thresholds is None else dict(self.thresholds),
            "shadow_only": self.shadow_only,
        }


_METRIC_TO_THRESHOLD = {
    "object_linear_speed_m_s": "max_object_linear_speed_m_s",
    "object_angular_speed_rad_s": "max_object_angular_speed_rad_s",
    "tcp_linear_speed_m_s": "max_tcp_linear_speed_m_s",
    "tcp_angular_speed_rad_s": "max_tcp_angular_speed_rad_s",
    "pad_origin_distance_rate_abs_m_s": "max_pad_origin_distance_rate_m_s",
    "arm_tracking_error_rad": "max_arm_tracking_error_rad",
    "gripper_object_penetration_m": "max_gripper_object_penetration_m",
    "object_lateral_point_impulse_norm_sum_ns": "max_object_lateral_impulse_norm_sum_ns",
    "support_relative_translation_error_m": "max_support_relative_translation_error_m",
}


def assess_release_readiness(
    metrics: Mapping[str, float],
    thresholds: ReleaseReadinessThresholds | None,
) -> ReleaseReadinessAssessment:
    """Classify one pre-release window without affecting execution.

    ``metrics`` should contain *worst-case values over a short pre-open window*,
    not only the final frame.  The only threshold-free hard rejection is a
    strict joint-limit excursion.  Everything else remains ``unknown`` until a
    complete calibrated threshold set is supplied.
    """

    observed: dict[str, float] = {}
    reasons: list[str] = []
    invalid: list[str] = []
    for key, raw in metrics.items():
        try:
            value = float(raw)
        except (TypeError, ValueError):
            invalid.append(str(key))
            continue
        if not math.isfinite(value):
            invalid.append(str(key))
            continue
        observed[str(key)] = value

    if invalid:
        return ReleaseReadinessAssessment(
            "unknown",
            ("non_finite_or_invalid_metrics:" + ",".join(sorted(invalid)),),
            observed,
            None if thresholds is None else thresholds.as_dict(),
        )

    joint_excess = observed.get("all_joint_limit_excess_rad", 0.0)
    if joint_excess > 0.0:
        reasons.append("joint_limit_excursion")

    if thresholds is None:
        if reasons:
            return ReleaseReadinessAssessment("not-ready", tuple(reasons), observed, None)
        return ReleaseReadinessAssessment(
            "unknown",
            ("release_thresholds_not_calibrated",),
            observed,
            None,
        )

    threshold_values = thresholds.as_dict()
    missing_metrics = sorted(set(_METRIC_TO_THRESHOLD) - set(observed))
    if missing_metrics:
        return ReleaseReadinessAssessment(
            "unknown",
            ("missing_metrics:" + ",".join(missing_metrics),),
            observed,
            threshold_values,
        )

    for metric_name, threshold_name in _METRIC_TO_THRESHOLD.items():
        value = abs(observed[metric_name])
        limit = threshold_values[threshold_name]
        if value > limit:
            reasons.append(
                f"{metric_name}_exceeds_{threshold_name}:"
                f"{value:.9g}>{limit:.9g}"
            )

    return ReleaseReadinessAssessment(
        "not-ready" if reasons else "ready",
        tuple(reasons) if reasons else ("all_calibrated_release_checks_pass",),
        observed,
        threshold_values,
    )
