#!/usr/bin/env python3
"""Synthetic, budget-matched PushT horizon ablation; NOT a paper reproduction.

The execution world and planning model are separate instances. Both are still
analytic quasi-static models: these results are not ManiSkill or real evidence.
No camera, external model service, GPU planner, or physical robot is contacted.
"""
from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rm75_app.scenarios.pusht import (  # noqa: E402
    PushTClosedLoopController, PushTControllerConfig, PushTGoal,
    PushTModelParameters, PushTMPC, PushTMPCConfig, PushTPose, PushTState,
    QuasiStaticPushTModel, SimulatedPushTWorld,
)


def candidates_for_budget(budget: int, horizon: int) -> int:
    if budget < 1 or horizon < 1 or budget % horizon:
        raise ValueError("budget must be positive and divisible by every horizon")
    return budget // horizon


def make_suite(count: int, seed: int) -> list[dict]:
    if count < 1:
        raise ValueError("episode count must be positive")
    rng = np.random.default_rng(seed)
    suite = []
    for index in range(count):
        position = rng.uniform(-0.07, 0.07, 2)
        yaw = float(rng.uniform(-0.65, 0.65))
        # Include orientation-only targets, not only straight pushes.
        goal_xy = position if index % 4 == 0 else position + rng.uniform(-0.08, 0.08, 2)
        suite.append({
            "episode_id": f"synthetic_{index:03d}",
            "initial_pose": [float(position[0]), float(position[1]), yaw],
            "goal_pose": [float(goal_xy[0]), float(goal_xy[1]), float(rng.uniform(-0.5, 0.5))],
            "execution_parameters": asdict(PushTModelParameters(
                friction=float(rng.uniform(0.25, 0.65)),
                translation_gain=float(rng.uniform(0.7, 0.95)),
                rotation_gain=float(rng.uniform(2.0, 3.8)),
                contact_efficiency=0.9,
            )),
        })
    return suite


def evaluate_case(case: dict, *, horizon: int, budget: int, max_steps: int,
                  planner_seed: int, parameter_mode: str) -> dict:
    if parameter_mode not in {"nominal", "oracle"}:
        raise ValueError("unknown parameter mode")
    true_parameters = PushTModelParameters(**case["execution_parameters"])
    world_model = QuasiStaticPushTModel(workspace_bounds_xy=(-0.45, 0.45, -0.45, 0.45))
    planning_model = QuasiStaticPushTModel(workspace_bounds_xy=(-0.45, 0.45, -0.45, 0.45))
    initial = PushTState(PushTPose(*case["initial_pose"]))
    world = SimulatedPushTWorld(world_model, initial, true_parameters=true_parameters)
    planner = PushTMPC(planning_model, PushTMPCConfig(
        horizon=horizon, candidate_sequences=candidates_for_budget(budget, horizon),
        seed=planner_seed,
    ))
    timings = []
    original_plan = planner.plan

    def measured_plan(*args):
        start = time.perf_counter()
        try:
            return original_plan(*args)
        finally:
            timings.append(time.perf_counter() - start)

    planner.plan = measured_plan
    controller = PushTClosedLoopController(
        world, world, planning_model, planner,
        parameters=true_parameters if parameter_mode == "oracle" else PushTModelParameters(),
        config=PushTControllerConfig(max_steps=max_steps, settle_time_s=0),
        sleep=lambda _seconds: None,
    )
    goal = PushTGoal(PushTPose(*case["goal_pose"]))
    report = controller.run(goal)
    errors = [planning_model.pose_error(t.plan.predicted_states[1], t.after.state)
              for t in report.transitions]
    return {
        "episode_id": case["episode_id"],
        "method": f"random_shooting_h{horizon}_{parameter_mode}",
        "success": report.success, "reason": report.reason,
        "privileged_parameters": parameter_mode == "oracle",
        "model_steps_per_decision": budget,
        "candidate_sequences": candidates_for_budget(budget, horizon),
        "planning_calls": len(timings), "planning_wall_times_s": timings,
        "model_steps_total": budget * len(timings),
        "executed_pushes": len(world.actions),
        "final_pose": report.final_observation.state.pose.vector().tolist(),
        "position_error_m": float(np.linalg.norm(report.final_observation.state.pose.xy - goal.pose.xy)),
        "one_step_prediction_errors_m": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=8)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--horizons", nargs="+", type=int, default=[1, 3])
    parser.add_argument("--budget", type=int, default=192)
    parser.add_argument("--max-steps", type=int, default=15)
    parser.add_argument("--parameter-modes", nargs="+", choices=["nominal", "oracle"], default=["nominal"])
    parser.add_argument("--source-commit", default="unrecorded-local-worktree")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.max_steps < 1 or len(set(args.horizons)) != len(args.horizons):
        parser.error("max-steps must be positive and horizons unique")
    for horizon in args.horizons:
        candidates_for_budget(args.budget, horizon)
    suite = make_suite(args.episodes, args.seed)
    rows = []
    # Paired frozen cases and order counterbalancing reduce one-sided timing drift.
    methods = [(h, mode) for h in args.horizons for mode in args.parameter_modes]
    for index, case in enumerate(suite):
        order = methods if index % 2 == 0 else list(reversed(methods))
        for horizon, mode in order:
            rows.append(evaluate_case(case, horizon=horizon, budget=args.budget,
                                      max_steps=args.max_steps, planner_seed=args.seed + index,
                                      parameter_mode=mode))
    summaries = {}
    for method in sorted({row["method"] for row in rows}):
        subset = [row for row in rows if row["method"] == method]
        timings = [value for row in subset for value in row["planning_wall_times_s"]]
        summaries[method] = {
            "episodes": len(subset), "successes": sum(row["success"] for row in subset),
            "p50_planning_wall_s": None if not timings else float(np.percentile(timings, 50)),
            "p95_planning_wall_s": None if not timings else float(np.percentile(timings, 95)),
            "planning_calls": len(timings),
        }
    payload = {
        "schema": "rm75.synthetic_horizon_ablation/v1",
        "evidence_level": "ANALYTIC_SYNTHETIC_ONLY",
        "source_commit": args.source_commit,
        "physical_commands": 0,
        "external_baselines_reproduced": [],
        "physics_simulator": None,
        "notes": ["Not CEM, MPPI, DINO-WM or PWTF.",
                  "No perfect-model claim: nominal planner does not receive execution parameters.",
                  "Step-call matching is not equal FLOPs/wall time; report both.",
                  "Analytic contacts are not certified physical push programs."],
        "environment": {"python": platform.python_version(), "numpy": np.__version__},
        "budget_per_decision": args.budget,
        "suite": suite, "rows": rows, "summary": summaries,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps(summaries, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
