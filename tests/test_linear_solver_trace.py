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
