#!/usr/bin/env python3
"""Local physics diagnostic only; no robot API and no initial-qpos teleport."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from run_unified_scenario import _write_json
from rm75_app.tasks.manipulation_plan import ManipulationPlan
from rm75_app.validation.maniskill_gate import run_maniskill_gate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiled-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    report = {"state": "NEEDS_REVIEW", "physical_commands": 0, "strict_timed_replay": True}
    try:
        frozen = json.loads((args.compiled_dir / "frozen_inputs.json").read_text())
        plan = ManipulationPlan.from_dict(frozen["plan"])
        result = run_maniskill_gate(
            plan, args.compiled_dir / "program/execution.json", args.output_dir,
            strict_timed_replay=True,
        )
        report["gate"] = asdict(result)
        report["success"] = result.passed
    except Exception as exc:
        report["exception"] = f"{type(exc).__name__}: {exc}"
    _write_json(args.output_dir / "strict_replay_summary.json", report)
    return 0 if report.get("success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
