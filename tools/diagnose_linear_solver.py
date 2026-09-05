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
    def __init__(self, terminal_only_ik=False, unconditioned_ik=False, cross_check_coarse=False):
        super().__init__()
        self.linear_calls = []
        self.terminal_only_ik = terminal_only_ik
        self.unconditioned_ik = unconditioned_ik
        self.cross_check_coarse = cross_check_coarse

    def plan_linear_candidates(self, request, **kwargs):
        planner = self._ensure_planner()
        rows = []
        entry = {"candidates": [c.candidate_id for c in request.candidates], "native_calls": rows}
        self.linear_calls.append(entry)
        original = planner.ik_solver.solve_pose
        def endpoint_ik(*args, **ik_kwargs):
            if "native_context" not in entry:
                coarse = self._coarse_ik_solver
                native_spheres = planner.ik_solver.kinematics.kinematics_config.link_spheres
                context = {"native_sphere_shape": list(native_spheres.shape),
                    "native_obstacles": {obj.name: self._collision_obstacle_enabled(planner.ik_solver.scene_collision_checker, obj.name)
                                         for obj in self._scene.objects}}
                if coarse is not None:
                    coarse_spheres = coarse.kinematics.kinematics_config.link_spheres
                    context["coarse_sphere_shape"] = list(coarse_spheres.shape)
                    if coarse_spheres.shape == native_spheres.shape:
                        context["max_sphere_parameter_difference"] = float((coarse_spheres - native_spheres).abs().max().item())
                    context["coarse_obstacles"] = {obj.name: self._collision_obstacle_enabled(coarse.scene_collision_checker, obj.name)
                                                   for obj in self._scene.objects}
                entry["native_context"] = context
            if self.terminal_only_ik:
                planner.ik_solver.update_tool_pose_criteria({
                    request.tool_frame: self._import_modules()["ToolPoseCriteria"]()})
            if self.unconditioned_ik:
                if len(args) > 1:
                    raise ValueError("diagnostic expects keyword current_state")
                ik_kwargs = {**ik_kwargs, "current_state": None}
            result = original(*args, **ik_kwargs)
            if self.cross_check_coarse and "coarse_same_goal" not in entry:
                coarse = self._coarse_ik_solver
                collision = coarse.scene_collision_checker
                saved = {obj.name: self._collision_obstacle_enabled(collision, obj.name) for obj in self._scene.objects}
                try:
                    for name, enabled in entry["native_context"]["native_obstacles"].items():
                        self._set_collision_obstacle_enabled(collision, name, enabled)
                    comparison = coarse.solve_pose(args[0], current_state=None, return_seeds=4)
                    entry["coarse_same_goal"] = {
                        "success_count": int(comparison.success.sum().item()),
                        "result_count": int(comparison.success.numel()),
                        "min_position_error_m": float(comparison.position_error.min().item()),
                        "max_position_error_m": float(comparison.position_error.max().item()),
                    }
                finally:
                    for name, enabled in saved.items():
                        self._set_collision_obstacle_enabled(collision, name, enabled)
            return result
        planner.ik_solver.solve_pose = endpoint_ik
        try:
            with trace_call(planner.ik_solver, "solve_pose", rows), \
                 trace_call(planner.trajopt_solver, "solve_pose", rows), \
                 trace_call(planner, "_get_graph_seed_trajectories", rows):
                result = super().plan_linear_candidates(request, **kwargs)
        finally:
            planner.ik_solver.solve_pose = original
        entry["statuses"] = [p.status for p in result.plans]
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--terminal-only-ik", action="store_true", help="Diagnostic only; retain linear TrajOpt conditions")
    modes.add_argument("--unconditioned-ik", action="store_true", help="Diagnostic only; omit current_state for goal IK, not for trajectory planning")
    modes.add_argument("--cross-check-coarse", action="store_true", help="Solve identical native goal with coarse IK under matching obstacle switches")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    plan = load_plan(args.plan)
    scene = load_task_scene(plan.scene_file)
    with TracedBackend(terminal_only_ik=args.terminal_only_ik, unconditioned_ik=args.unconditioned_ik,
                      cross_check_coarse=args.cross_check_coarse) as backend:
        native = backend._ensure_planner()
        scene.set_joints(tuple(native.joint_names), native.default_joint_state.position.detach().cpu().numpy().reshape(-1)[:7])
        builder, _ = _scene_builder(Path(plan.scene_file))
        result = PickPlaceCoordinator(backend, _NoopExecutor()).run(builder(plan.atoms[0], scene))
        _write_json(args.output, {"fixture_only": True, "success": result.success,
            "terminal_only_ik": args.terminal_only_ik,
            "unconditioned_ik": args.unconditioned_ik,
            "cross_check_coarse": args.cross_check_coarse,
            "failure_stage": result.failure_stage, "linear_calls": backend.linear_calls,
            "diagnostics": result.diagnostics})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
