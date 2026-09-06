from __future__ import annotations

from rm75_app.scenarios.release_readiness import (
    ReleaseReadinessThresholds,
    assess_release_readiness,
)


def _metrics():
    return {
        "all_joint_limit_excess_rad": 0.0,
        "object_linear_speed_m_s": 0.01,
        "object_angular_speed_rad_s": 0.02,
        "tcp_linear_speed_m_s": 0.01,
        "tcp_angular_speed_rad_s": 0.02,
        "pad_origin_distance_rate_abs_m_s": 0.001,
        "arm_tracking_error_rad": 0.01,
        "gripper_object_penetration_m": 0.0001,
        "object_lateral_point_impulse_norm_sum_ns": 0.01,
        "support_relative_translation_error_m": 0.001,
    }


def _thresholds():
    return ReleaseReadinessThresholds(
        max_object_linear_speed_m_s=0.02,
        max_object_angular_speed_rad_s=0.04,
        max_tcp_linear_speed_m_s=0.02,
        max_tcp_angular_speed_rad_s=0.04,
        max_pad_origin_distance_rate_m_s=0.002,
        max_arm_tracking_error_rad=0.02,
        max_gripper_object_penetration_m=0.0002,
        max_object_lateral_impulse_norm_sum_ns=0.02,
        max_support_relative_translation_error_m=0.002,
    )


def test_release_readiness_stays_unknown_without_calibration() -> None:
    result = assess_release_readiness(_metrics(), None)
    assert result.state == "unknown"
    assert "release_thresholds_not_calibrated" in result.reasons
    assert result.shadow_only is True


def test_joint_limit_excursion_is_not_ready_without_other_thresholds() -> None:
    metrics = _metrics()
    metrics["all_joint_limit_excess_rad"] = 1.0e-4
    result = assess_release_readiness(metrics, None)
    assert result.state == "not-ready"
    assert "joint_limit_excursion" in result.reasons


def test_calibrated_release_window_can_be_ready() -> None:
    result = assess_release_readiness(_metrics(), _thresholds())
    assert result.state == "ready"
    assert result.reasons == ("all_calibrated_release_checks_pass",)


def test_calibrated_release_window_reports_exceeded_metric() -> None:
    metrics = _metrics()
    metrics["object_linear_speed_m_s"] = 0.03
    result = assess_release_readiness(metrics, _thresholds())
    assert result.state == "not-ready"
    assert any("object_linear_speed_m_s" in reason for reason in result.reasons)
