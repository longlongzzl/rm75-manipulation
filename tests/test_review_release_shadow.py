from pathlib import Path
import pytest


@pytest.mark.parametrize("excess,expected", [(0., "unknown"), (0.001, "not-ready")])
def test_uncalibrated_shadow_never_claims_ready(monkeypatch, excess, expected):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from record_original_release_evidence import shadow_readiness
    sample = {"limit_excess_rad": [excess], "frame_id": 20, "simulation_time_s": 0.1}
    result = shadow_readiness(sample)
    assert result["state"] == expected
    assert result["shadow_only"]
    assert result["frame_id"] == 20
    assert sample["limit_excess_rad"] == [excess]


def test_missing_observation_is_unknown(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from record_original_release_evidence import shadow_readiness
    assert shadow_readiness(None)["state"] == "unknown"
