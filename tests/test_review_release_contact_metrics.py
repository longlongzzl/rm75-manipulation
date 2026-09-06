from pathlib import Path


def test_penetration_is_gripper_specific_and_lateral_impulses_do_not_cancel(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from summarize_release_evidence import contact_metrics
    contacts = [{"body_a": "gripper_Left_Support_Link", "body_b": "scene-0_carriot",
                 "points": [{"separation_m": -0.002, "impulse_ns": [1,0,0]},
                            {"separation_m": 0.001, "impulse_ns": [-1,0,0]}]},
                {"body_a": "scene-0_shuazi", "body_b": "scene-0_carriot",
                 "points": [{"separation_m": -0.01, "impulse_ns": [0,0,3]}]}]
    assert contact_metrics(contacts, "carriot") == (0.002, 2.0)
    assert contact_metrics(contacts, "tennis") == (0., 0.)
