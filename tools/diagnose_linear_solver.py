#!/usr/bin/env python3
"""Trace native solver outcomes without altering arguments, returns or settings."""
import argparse
from contextlib import contextmanager
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark_grasp_relation_screen import _scene_builder, _NoopExecutor
from run_unified_scenario import _write_json
from rm75_app.orchestration.multi_object_executor import load_task_scene
from rm75_app.pickplace.coordinator import PickPlaceCoordinator
from rm75_app.planning.backends.curobo2 import Curobo2Backend
from rm75_app.tasks.manipulation_plan import load_plan


@contextmanager
def trace_call(owner, name, rows):
    original = getattr(owner, name)
    def traced(*args, **kwargs):
        result = original(*args, **kwargs)
        row = {"owner": type(owner).__name__, "call": name, "returned_none": result is None}
        success = getattr(result, "success", None)
        if success is not None:
            values = success.detach().cpu().numpy()
            row.update(success_count=int(values.sum()), result_count=int(values.size))
        for field in ("feasible", "position_error", "rotation_error", "position_tolerance", "orientation_tolerance"):
            value = getattr(result, field, None)
            if value is not None:
                row[field] = value.detach().cpu().numpy().tolist() if hasattr(value, "detach") else value
        rows.append(row)
        return result
    setattr(owner, name, traced)
    try:
        yield
    finally:
        setattr(owner, name, original)


class TracedBackend(Curobo2Backend):
    def __init__(self, terminal_only_ik=False):
        super().__init__()
        self.linear_calls = []
        self.terminal_only_ik = terminal_only_ik

    def plan_linear_candidates(self, request, **kwargs):
        planner = self._ensure_planner()
        rows = []
        entry = {"candidates": [c.candidate_id for c in request.candidates], "native_calls": rows}
        self.linear_calls.append(entry)
        original = planner.ik_solver.solve_pose
        def endpoint_ik(*args, **ik_kwargs):
            planner.ik_solver.update_tool_pose_criteria({
                request.tool_frame: self._import_modules()["ToolPoseCriteria"]()})
            return original(*args, **ik_kwargs)
        if self.terminal_only_ik:
            planner.ik_solver.solve_pose = endpoint_ik
        try:
            with trace_call(planner.ik_solver, "solve_pose", rows), \
                 trace_call(planner.trajopt_solver, "solve_pose", rows), \
                 trace_call(planner, "_get_graph_seed_trajectories", rows):
                result = super().plan_linear_candidates(request, **kwargs)
        finally:
            if self.terminal_only_ik:
                planner.ik_solver.solve_pose = original
        entry["statuses"] = [p.status for p in result.plans]
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--terminal-only-ik", action="store_true", help="Diagnostic only; retain linear TrajOpt conditions")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    plan = load_plan(args.plan)
    scene = load_task_scene(plan.scene_file)
    with TracedBackend(terminal_only_ik=args.terminal_only_ik) as backend:
        native = backend._ensure_planner()
        scene.set_joints(tuple(native.joint_names), native.default_joint_state.position.detach().cpu().numpy().reshape(-1)[:7])
        builder, _ = _scene_builder(Path(plan.scene_file))
        result = PickPlaceCoordinator(backend, _NoopExecutor()).run(builder(plan.atoms[0], scene))
        _write_json(args.output, {"fixture_only": True, "success": result.success,
            "terminal_only_ik": args.terminal_only_ik,
            "failure_stage": result.failure_stage, "linear_calls": backend.linear_calls,
            "diagnostics": result.diagnostics})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
