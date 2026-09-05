#!/usr/bin/env python3
"""Opt-in time-scaled simulation diagnostic; source program remains unchanged."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rm75_app.execution.trajectory_executor import sample_timed_joint_path
from rm75_app.runtime.curobo2_sim_replay import load_replay_events
from rm75_app.tasks.manipulation_plan import ManipulationPlan
from rm75_app.validation.maniskill_gate import run_maniskill_gate


def scale_events(events, factor):
    if not np.isfinite(factor) or factor < 1:
        raise ValueError("time factor must be finite and at least one")
    output, audit = [], []
    for event in events:
        item = dict(event)
        if item.get("type") == "trajectory":
            source = item["trajectory"]
            # Reuse strict dt/point validation before scaling and executing.
            sample_timed_joint_path(source, 0.05)
            dt = np.asarray(source.dt, dtype=float)
            scaled = replace(source, positions=source.positions.copy(), dt=dt * factor)
            item["trajectory"] = scaled
            audit.append({"stage": item["stage"], "source_dt": dt.tolist(),
                          "scaled_dt": np.asarray(scaled.dt).tolist(),
                          "points": len(source.positions),
                          "positions_sha256": hashlib.sha256(source.positions.tobytes()).hexdigest(),
                          "positions_unchanged": bool(np.array_equal(source.positions, scaled.positions))})
        output.append(item)
    return output, audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiled-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--time-factor", type=float, required=True)
    args = parser.parse_args()
    manifest = args.compiled_dir / "program/execution.json"
    events, audit = scale_events(load_replay_events(manifest), args.time_factor)
    frozen = json.loads((args.compiled_dir / "frozen_inputs.json").read_text())
    plan = ManipulationPlan.from_dict(frozen["plan"])
    args.output_dir.mkdir(parents=True, exist_ok=False)
    report = {"state": "NEEDS_REVIEW", "physical_commands": 0,
              "time_factor": args.time_factor, "trajectory_audit": audit,
              "source_manifest": str(manifest),
              "source_manifest_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest(),
              "gripper_dwell_changed": False, "production_behavior_changed": False,
              "limitations": ["Retimed diagnostic, not original program replay", "Not full SIM or REAL approval"]}
    try:
        with patch("rm75_app.validation.maniskill_gate.load_replay_events", return_value=events):
            result = run_maniskill_gate(plan, manifest, args.output_dir, strict_timed_replay=True)
        report["task_result"] = asdict(result)
        report["task_success"] = result.passed
    except Exception as exc:
        report["exception"] = f"{type(exc).__name__}: {exc}"
    (args.output_dir / "diagnostic_summary.json").write_text(json.dumps(report, indent=2))
    return 0 if report.get("task_success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
