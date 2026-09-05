#!/usr/bin/env python3
"""Opt-in simulated release control experiment; never a production replay pass.

For explicitly numbered openings, hold the measured arm configuration instead
of continuing to track the planned endpoint. Restore the planned target after
that dwell. All trajectory samples, other gripper events and dwell lengths stay
unchanged. This changes commands and must not be labeled unmodified replay.
"""
import argparse
from contextlib import ExitStack, contextmanager
from dataclasses import asdict
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rm75_app.execution.trajectory_executor import ManiSkillTrajectoryExecutor
from rm75_app.tasks.manipulation_plan import ManipulationPlan
from rm75_app.validation.maniskill_gate import run_maniskill_gate


@contextmanager
def observed_open_control(executor_class, open_index, records):
    if open_index < 1:
        raise ValueError("open index must be positive")
    original = executor_class.set_gripper
    counts = {}

    def set_gripper(executor, closed):
        if closed:
            return original(executor, closed)
        counts[executor] = counts.get(executor, 0) + 1
        if counts[executor] != open_index:
            return original(executor, closed)
        if executor.control_dt is None or executor._last_commanded_target is None:
            raise ValueError("selected opening requires timed trajectory endpoint")
        planned = executor._last_commanded_target
        observed = np.asarray(executor.demo.current_arm_qpos(), dtype=float).copy()
        if observed.shape != planned.shape or not np.isfinite(observed).all():
            raise ValueError("invalid observed arm configuration")
        records.append({"open_index": open_index, "planned": planned.tolist(),
                        "observed": observed.tolist(),
                        "max_command_delta_rad": float(np.max(np.abs(planned - observed)))})
        try:
            executor._last_commanded_target = observed
            return original(executor, closed)
        finally:
            executor._last_commanded_target = planned

    with patch.object(executor_class, "set_gripper", set_gripper):
        yield


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiled-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--open-index", type=int, action="append", required=True)
    args = parser.parse_args()
    if len(set(args.open_index)) != len(args.open_index) or min(args.open_index) < 1:
        parser.error("opening indices must be distinct positive integers")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    frozen = json.loads((args.compiled_dir / "frozen_inputs.json").read_text())
    plan = ManipulationPlan.from_dict(frozen["plan"])
    report = {"state": "NEEDS_REVIEW", "physical_commands": 0,
              "diagnostic_command_override": True, "production_behavior_changed": False,
              "limitations": ["Not unmodified program replay", "Not SIM_VERIFIED or real trial approval"],
              "overrides": []}
    try:
        with ExitStack() as stack:
            for index in args.open_index:
                stack.enter_context(observed_open_control(
                    ManiSkillTrajectoryExecutor, index, report["overrides"]))
            result = run_maniskill_gate(plan, args.compiled_dir / "program/execution.json",
                                       args.output_dir, strict_timed_replay=True)
        report["task_result"] = asdict(result)
        report["task_success"] = result.passed
        if sorted(row["open_index"] for row in report["overrides"]) != sorted(args.open_index):
            raise ValueError("selected openings were not reached exactly once each")
    except Exception as exc:
        report["exception"] = f"{type(exc).__name__}: {exc}"
    (args.output_dir / "diagnostic_summary.json").write_text(json.dumps(report, indent=2))
    return 0 if report.get("task_success") and not report.get("exception") else 2


if __name__ == "__main__":
    raise SystemExit(main())
