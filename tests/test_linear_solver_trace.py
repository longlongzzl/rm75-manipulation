from pathlib import Path

import pytest


def test_trace_preserves_arguments_return_identity_and_restores(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_linear_solver import trace_call
    calls = []
    sentinel = object()
    class Solver:
        def solve(self, *args, **kwargs):
            calls.append((args, kwargs))
            return sentinel
    solver = Solver()
    original = solver.solve
    rows = []
    with trace_call(solver, "solve", rows):
        assert solver.solve(1, seed=3) is sentinel
    assert solver.solve == original
    assert calls == [((1,), {"seed": 3})]
    assert rows == [{"owner": "Solver", "call": "solve", "returned_none": False}]


def test_trace_restores_after_solver_error(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_linear_solver import trace_call
    class Solver:
        def solve(self):
            raise RuntimeError("native failure")
    solver = Solver()
    original = solver.solve
    with pytest.raises(RuntimeError, match="native failure"):
        with trace_call(solver, "solve", []):
            solver.solve()
    assert solver.solve == original


def test_trace_retains_result_metric_fields_without_reinterpreting_them(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_linear_solver import trace_call
    result = SimpleNamespace(position_error=0.04, rotation_error=0.03,
                             position_tolerance=0.0, orientation_tolerance=0.0)
    owner = SimpleNamespace(solve=lambda: result)
    rows = []
    with trace_call(owner, "solve", rows):
        assert owner.solve() is result
    assert rows[0]["position_error"] == 0.04
    assert rows[0]["position_tolerance"] == 0.0


@pytest.mark.parametrize("enabled", [False, True])
def test_failed_rescreen_cache_eviction_is_diagnostic_only(monkeypatch, enabled):
    from types import SimpleNamespace
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_linear_solver import TracedBackend, Curobo2Backend
    monkeypatch.setattr(Curobo2Backend, "_prepare_pose_candidates_with_solver",
                        lambda self, candidates, **kwargs: {"grasp_a": {"success": False}})
    backend = TracedBackend.__new__(TracedBackend)
    backend.reject_stale_cache = enabled
    backend.screen_trace = []
    backend._pose_ik_cache = {"grasp_a": object()}
    backend._pose_cache_key = lambda candidate: candidate.candidate_id
    candidate = SimpleNamespace(candidate_id="grasp_a", pose=SimpleNamespace(as_curobo_list=lambda: [0]*7))
    backend._prepare_pose_candidates_with_solver((candidate,))
    assert ("grasp_a" in backend._pose_ik_cache) is (not enabled)
