#!/usr/bin/env python3
"""Export visual evidence for every failed language-benchmark case."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path
from typing import Any


DEFAULT_PYTHON = "/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark-dir", type=Path, required=True)
    parser.add_argument("--corpus-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--python", default=DEFAULT_PYTHON)
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args()


def _nearest_grasp_candidate(report: dict[str, Any]) -> str | None:
    for gate in report.get("gates") or []:
        if gate.get("gate") != "curobo2":
            continue
        for check in gate.get("checks") or []:
            for attempt in check.get("attempts") or []:
                diagnostics = attempt.get("diagnostics") or {}
                selected = (attempt.get("artifacts") or {}).get("selected_grasp")
                if selected:
                    return str(selected)
                endpoint = (diagnostics.get("relation_screen") or {}).get(
                    "grasp_endpoint_summary"
                ) or {}
                nearest = endpoint.get("nearest_failures") or []
                if nearest and nearest[0].get("candidate_id"):
                    return str(nearest[0]["candidate_id"])
    return None


def _failure_details(report: dict[str, Any]) -> tuple[str, str]:
    for gate in report.get("gates") or []:
        if gate.get("status") in {"passed", "skipped"}:
            continue
        gate_name = str(gate.get("gate") or "unknown")
        checks = gate.get("checks") or []
        failed = next(
            (item for item in checks if item.get("status") == "failed"),
            checks[0] if checks else {},
        )
        attempts = failed.get("attempts") or []
        stage = next(
            (
                str(item.get("failure_stage"))
                for item in reversed(attempts)
                if item.get("failure_stage")
            ),
            "validation",
        )
        message = str(
            failed.get("message")
            or gate.get("summary")
            or gate.get("error")
            or "failed"
        )
        return f"{gate_name}:{stage}", message
    return "unknown", "report did not identify a failed gate"


def _has_trajectory(manifest: Path) -> bool:
    if not manifest.is_file():
        return False
    try:
        events = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    return any(item.get("type") == "trajectory" for item in events)


def main() -> int:
    args = parse_args()
    benchmark = args.benchmark_dir.expanduser().resolve()
    corpus = args.corpus_dir.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    summary = json.loads((benchmark / "summary.json").read_text(encoding="utf-8"))
    failed_rows = [item for item in summary.get("cases") or [] if not item.get("passed")]
    records: list[dict[str, Any]] = []
    for index, row in enumerate(failed_rows, start=1):
        case_no = int(row["case"])
        case_name = f"case_{case_no:02d}"
        case_output = output / case_name
        case_output.mkdir(parents=True, exist_ok=True)
        report_file = benchmark / case_name / "three_gate_report.json"
        report = json.loads(report_file.read_text(encoding="utf-8"))
        failure_kind, failure_message = _failure_details(report)
        candidate_id = (
            _nearest_grasp_candidate(report)
            if failure_kind.startswith("curobo2:")
            else None
        )
        plan_file = corpus / case_name / "manipulation_plan.json"
        execution_file = benchmark / case_name / "execution.json"
        # Current planner failures have no trajectory and therefore cannot be
        # truthfully replayed. Export a collision-aware four-view snapshot.
        mode = "video" if _has_trajectory(execution_file) else "snapshot"
        image_file = case_output / "maniskill_scene_preview.png"
        command = [
            str(args.python),
            "-m",
            "rm75_app.runtime.maniskill_scene_preview",
            "--plan",
            str(plan_file),
            "--output-dir",
            str(case_output),
            "--snapshot",
        ]
        if candidate_id:
            command.extend(["--candidate-id", candidate_id])
        started = time.perf_counter()
        if args.resume and image_file.is_file():
            returncode = 0
            stdout = "resumed existing snapshot"
            stderr = ""
        else:
            try:
                completed = subprocess.run(
                    command,
                    cwd=str(Path(__file__).parents[1]),
                    text=True,
                    capture_output=True,
                    timeout=float(args.timeout_s),
                    check=False,
                )
                returncode = int(completed.returncode)
                stdout = completed.stdout or ""
                stderr = completed.stderr or ""
                if returncode != 0 and candidate_id:
                    fallback = subprocess.run(
                        command[:-2],
                        cwd=str(Path(__file__).parents[1]),
                        text=True,
                        capture_output=True,
                        timeout=float(args.timeout_s),
                        check=False,
                    )
                    stdout += "\n[candidate fallback]\n" + (fallback.stdout or "")
                    stderr += "\n[candidate fallback]\n" + (fallback.stderr or "")
                    returncode = int(fallback.returncode)
                    if returncode == 0:
                        candidate_id = None
            except subprocess.TimeoutExpired as exc:
                returncode = 124
                stdout = str(exc.stdout or "")
                stderr = f"snapshot timed out after {float(args.timeout_s):.1f}s"
        elapsed = time.perf_counter() - started
        (case_output / "export.stdout.log").write_text(stdout, encoding="utf-8")
        (case_output / "export.stderr.log").write_text(stderr, encoding="utf-8")
        record = {
            "case": case_no,
            "command": row.get("command"),
            "failure_kind": failure_kind,
            "failure_message": failure_message,
            "planner_internal_time_s": row.get("planner_internal_time_s"),
            "over_5s": float(row.get("planner_internal_time_s") or 0.0) > 5.0,
            "evidence_mode": "snapshot" if mode == "snapshot" else "snapshot_no_replay_adapter",
            "candidate_id": candidate_id,
            "image": str(image_file) if image_file.is_file() else None,
            "scene_spec": str(case_output / "maniskill_preview_scene.json"),
            "export_time_s": elapsed,
            "returncode": returncode,
            "error": None if returncode == 0 else (stderr or stdout)[-2000:],
        }
        records.append(record)
        (output / "evidence_manifest.json").write_text(
            json.dumps(
                {
                    "source_benchmark": str(benchmark),
                    "hard_planner_budget_s": 5.0,
                    "failed_case_count": len(failed_rows),
                    "records": records,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        print(
            f"[{index:02d}/{len(failed_rows):02d}] {case_name} "
            f"{'OK' if returncode == 0 and image_file.is_file() else 'ERROR'} "
            f"{failure_kind} export={elapsed:.2f}s",
            flush=True,
        )
    return 0 if records and all(item["image"] for item in records) else 2


if __name__ == "__main__":
    raise SystemExit(main())
