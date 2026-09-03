#!/usr/bin/env python3
"""Batch the frozen natural-language manipulation plans through validation gates."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import subprocess
import time
from collections import Counter
from pathlib import Path
from typing import Any


DEFAULT_PYTHON = "/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--corpus-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--through", choices=("geometry", "curobo2", "maniskill"), default="curobo2")
    parser.add_argument("--python", default=DEFAULT_PYTHON)
    parser.add_argument("--cases", nargs="*", type=int, help="Optional one-based case numbers")
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args()


def percentile(values: list[float], ratio: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * ratio
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - rank) + ordered[high] * (rank - low)


def nested_diagnostics(gate: dict[str, Any]) -> list[dict[str, Any]]:
    found: list[dict[str, Any]] = []
    for check in gate.get("checks") or []:
        for attempt in check.get("attempts") or []:
            diagnostics = attempt.get("diagnostics") or {}
            if diagnostics:
                found.append(diagnostics)
    return found


def classify_failure(report: dict[str, Any]) -> tuple[str, str]:
    for gate in report.get("gates") or []:
        status = gate.get("status")
        if status == "passed":
            continue
        if status == "skipped":
            continue
        gate_name = str(gate.get("gate") or "unknown")
        if status == "error":
            return f"{gate_name}:error", str(gate.get("error") or gate.get("summary") or "worker error")
        checks = gate.get("checks") or []
        failed = next((item for item in checks if item.get("status") == "failed"), checks[0] if checks else {})
        attempts = failed.get("attempts") or []
        stage = next(
            (str(item.get("failure_stage")) for item in reversed(attempts) if item.get("failure_stage")),
            "validation",
        )
        message = str(failed.get("message") or gate.get("summary") or "failed")
        if gate_name == "maniskill":
            message = str(failed.get("message") or message)
        return f"{gate_name}:{stage}", message
    return "unknown", "report did not identify a failed gate"


def summarize_case(case_no: int, case_dir: Path, report: dict[str, Any], wall_s: float) -> dict[str, Any]:
    manifest = json.loads((case_dir / "manifest.json").read_text(encoding="utf-8"))
    plan = json.loads((case_dir / "manipulation_plan.json").read_text(encoding="utf-8"))
    gates = {str(item["gate"]): item for item in report.get("gates") or []}
    diagnostics = nested_diagnostics(gates.get("curobo2") or {})
    screens = [float(item.get("relation_screen", {}).get("screen_time_s", 0.0)) for item in diagnostics]
    plans = [float(item.get("timing", {}).get("segmented_plan_time_s", 0.0)) for item in diagnostics]
    passed = bool(report.get("passed"))
    failure_kind, failure_message = ("", "") if passed else classify_failure(report)
    return {
        "case": case_no,
        "command": manifest.get("command", ""),
        "atom_count": len(plan.get("atoms") or []),
        "passed": passed,
        "geometry": (gates.get("geometry") or {}).get("status", "not_run"),
        "curobo2": (gates.get("curobo2") or {}).get("status", "not_run"),
        "maniskill": (gates.get("maniskill") or {}).get("status", "not_run"),
        "wall_time_s": wall_s,
        "relation_screen_time_s": sum(screens),
        "segmented_plan_time_s": sum(plans),
        "planner_internal_time_s": sum(screens) + sum(plans),
        "failure_kind": failure_kind,
        "failure_message": failure_message,
    }


def aggregate(rows: list[dict[str, Any]], through: str) -> dict[str, Any]:
    wall = [float(row["wall_time_s"]) for row in rows]
    internal = [float(row["planner_internal_time_s"]) for row in rows if row["curobo2"] != "not_run"]
    passed = sum(bool(row["passed"]) for row in rows)
    gate_counts = {
        gate: dict(Counter(str(row[gate]) for row in rows))
        for gate in ("geometry", "curobo2", "maniskill")
    }
    return {
        "through": through,
        "case_count": len(rows),
        "atom_count": sum(int(row["atom_count"]) for row in rows),
        "passed": passed,
        "failed": len(rows) - passed,
        "success_rate": passed / len(rows) if rows else 0.0,
        "gate_status_counts": gate_counts,
        "failure_reason_counts": dict(Counter(row["failure_kind"] for row in rows if row["failure_kind"])),
        "wall_time_s": stats(wall),
        "planner_internal_time_s": stats(internal),
    }


def stats(values: list[float]) -> dict[str, float | None]:
    return {
        "total": sum(values),
        "mean": statistics.fmean(values) if values else None,
        "median": statistics.median(values) if values else None,
        "p90": percentile(values, 0.90),
        "max": max(values) if values else None,
    }


def write_outputs(output: Path, rows: list[dict[str, Any]], through: str) -> None:
    output.mkdir(parents=True, exist_ok=True)
    summary = {"aggregate": aggregate(rows, through), "cases": rows}
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    if rows:
        with (output / "cases.csv").open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def main() -> int:
    args = parse_args()
    corpus = args.corpus_dir.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    selected = set(args.cases or [])
    case_dirs = sorted(path for path in corpus.glob("case_*"))
    rows: list[dict[str, Any]] = []
    for index, case_dir in enumerate(case_dirs, 1):
        case_no = int(case_dir.name.rsplit("_", 1)[-1])
        if selected and case_no not in selected:
            continue
        case_output = output / case_dir.name
        report_file = case_output / "three_gate_report.json"
        timing_file = case_output / "benchmark_timing.json"
        if args.resume and report_file.is_file() and timing_file.is_file():
            report = json.loads(report_file.read_text(encoding="utf-8"))
            wall_s = float(json.loads(timing_file.read_text(encoding="utf-8"))["wall_time_s"])
        else:
            case_output.mkdir(parents=True, exist_ok=True)
            command = [
                args.python,
                "-m",
                "rm75_app",
                "task-validate",
                "--",
                "--plan",
                str(case_dir / "manipulation_plan.json"),
                "--through",
                args.through,
                "--output-dir",
                str(case_output),
            ]
            started = time.perf_counter()
            completed = subprocess.run(command, text=True, capture_output=True, check=False)
            wall_s = time.perf_counter() - started
            (case_output / "worker.stdout.log").write_text(completed.stdout or "", encoding="utf-8")
            (case_output / "worker.stderr.log").write_text(completed.stderr or "", encoding="utf-8")
            timing_file.write_text(
                json.dumps({"wall_time_s": wall_s, "returncode": completed.returncode}, indent=2),
                encoding="utf-8",
            )
            if not report_file.is_file():
                report = {
                    "passed": False,
                    "gates": [{"gate": args.through, "status": "error", "error": (completed.stderr or completed.stdout)[-4000:]}],
                }
                report_file.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            else:
                report = json.loads(report_file.read_text(encoding="utf-8"))
        row = summarize_case(case_no, case_dir, report, wall_s)
        rows.append(row)
        write_outputs(output, rows, args.through)
        print(
            f"[{len(rows):02d}/{len(selected) if selected else len(case_dirs):02d}] "
            f"case_{case_no:02d} {'PASS' if row['passed'] else 'FAIL'} "
            f"wall={wall_s:.2f}s internal={row['planner_internal_time_s']:.2f}s "
            f"{row['failure_kind']}",
            flush=True,
        )
    return 0 if rows and all(row["passed"] for row in rows) else 2


if __name__ == "__main__":
    raise SystemExit(main())
