from __future__ import annotations

import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
PYTHON = Path(r"D:\MiniConda\envs\sim2real-curobo\python.exe")
SCRIPT = ROOT / "standard_four_wall_retry_build.py"
OUT_ROOT = ROOT / "repro_runs" / "v0_9_release_first_pair_prefilter_random10_3141_3150_20260516_01"
SEEDS = list(range(3141, 3151))


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def classify_case(case_dir: Path, seed: int, elapsed: float, returncode: int) -> dict[str, Any]:
    report_path = case_dir / "standard_build_report.json"
    report = read_json(report_path)
    final = report.get("final", {}) if isinstance(report.get("final"), dict) else {}
    attempts = report.get("attempts", []) if isinstance(report.get("attempts"), list) else []
    success = bool(final.get("success")) and returncode == 0
    latest_video = final.get("latest_video")
    if not latest_video:
        for attempt in reversed(attempts):
            if isinstance(attempt, dict) and attempt.get("video"):
                latest_video = attempt.get("video")
                break
    successful_attempts = [a for a in attempts if isinstance(a, dict) and a.get("success")]
    last_attempt = attempts[-1] if attempts and isinstance(attempts[-1], dict) else {}
    failed_attempt = next(
        (a for a in reversed(attempts) if isinstance(a, dict) and not a.get("success")),
        last_attempt,
    )
    mode = "unknown"
    if success:
        if successful_attempts:
            mode = str(successful_attempts[-1].get("mode") or "direct_multi")
        elif final.get("completed_roles"):
            mode = "direct_multi"
    failure = failed_attempt.get("classification", {}) if isinstance(failed_attempt.get("classification"), dict) else {}
    offsets: dict[str, Any] = {}
    for attempt in attempts:
        summary_path = attempt.get("summary_path") if isinstance(attempt, dict) else None
        summary = read_json(Path(summary_path)) if summary_path else {}
        final_summary = summary.get("final", {}) if isinstance(summary.get("final"), dict) else {}
        jitter = final_summary.get("initial_actor_jitter", {}) if isinstance(final_summary.get("initial_actor_jitter"), dict) else {}
        if isinstance(jitter.get("offsets"), dict):
            offsets = jitter["offsets"]
            break
    return {
        "seed": seed,
        "case_dir": str(case_dir),
        "returncode": returncode,
        "success": success,
        "mode": mode,
        "elapsed_sec": round(elapsed, 3),
        "report": str(report_path) if report_path.exists() else None,
        "video": latest_video,
        "completed_roles": final.get("completed_roles", []),
        "failure": failure,
        "offsets": offsets,
    }


def aggregate(cases: list[dict[str, Any]]) -> dict[str, Any]:
    finished = [case for case in cases if "success" in case]
    successes = [case for case in finished if case.get("success")]
    failures = [case for case in finished if not case.get("success")]
    elapsed = [float(case.get("elapsed_sec", 0.0)) for case in finished]
    direct = [case for case in successes if str(case.get("mode", "")).lower() in {"", "none", "direct_multi"}]
    retry = [case for case in successes if case not in direct]
    return {
        "out_root": str(OUT_ROOT),
        "started_at": cases[0].get("started_at") if cases else None,
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "finished_cases": len(finished),
        "total_cases": len(SEEDS),
        "success_count": len(successes),
        "failure_count": len(failures),
        "success_rate": (len(successes) / len(finished)) if finished else 0.0,
        "average_elapsed_sec": (sum(elapsed) / len(elapsed)) if elapsed else None,
        "min_elapsed_sec": min(elapsed) if elapsed else None,
        "max_elapsed_sec": max(elapsed) if elapsed else None,
        "direct_success_count": len(direct),
        "retry_success_count": len(retry),
        "direct_average_elapsed_sec": (
            sum(float(case["elapsed_sec"]) for case in direct) / len(direct) if direct else None
        ),
        "retry_average_elapsed_sec": (
            sum(float(case["elapsed_sec"]) for case in retry) / len(retry) if retry else None
        ),
        "cases": finished,
    }


def main() -> int:
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    progress_path = OUT_ROOT / "progress.json"
    aggregate_path = OUT_ROOT / "aggregate.json"
    cases: list[dict[str, Any]] = []
    started_at = datetime.now().isoformat(timespec="seconds")
    for index, seed in enumerate(SEEDS, start=1):
        case_dir = OUT_ROOT / f"case_{index:02d}_seed_{seed}"
        case_dir.mkdir(parents=True, exist_ok=True)
        case_record: dict[str, Any] = {
            "index": index,
            "seed": seed,
            "case_dir": str(case_dir),
            "started_at": started_at if index == 1 else datetime.now().isoformat(timespec="seconds"),
            "status": "running",
        }
        cases.append(case_record)
        write_json(progress_path, {"status": "running", "cases": cases, "aggregate": aggregate(cases)})
        command = [
            str(PYTHON),
            str(SCRIPT),
            "--out-dir",
            str(case_dir),
            "--no-unique-out-dir",
            "--strategy-preset",
            "pair_first_robust_fast_v1",
            "--execution-mode",
            "direct-first",
            "--roles",
            "right_wall,back_wall,left_wall,front_wall",
            "--role-order-policy",
            "given",
            "--max-attempts-per-role",
            "3",
            "--use-cached-pair-release",
            "--wall-release-profile",
            "fixed_top_down",
            "--fixed-top-down-open-gap",
            "0.006",
            "--retry-extra-tilt-start-role-attempt",
            "3",
            "--retry-wall-grasp-extra-tilt-degs",
            "15,-15,20,-20,30,-30,45,-45",
            "--retry-wall-grasp-extra-tilt-max-abs-deg",
            "45",
            "--retry-fixed-top-down-extra-tilt-degs",
            "15,-15,20,-20,30,-30,45,-45",
            "--retry-fixed-top-down-extra-tilt-max-abs-deg",
            "45",
            "--couple-wall-grasp-release-tilt",
            "--runtime-collision-monitor",
            "--initial-actor-jitter-xy",
            "0.05",
            "--initial-actor-jitter-seed",
            str(seed),
            "--initial-actor-jitter-roles",
            "right_wall,back_wall,left_wall,front_wall",
            "--initial-actor-jitter-min-start-distance",
            "0.045",
            "--initial-actor-jitter-min-target-distance",
            "0.025",
            "--initial-actor-jitter-max-sample-attempts",
            "300",
            "--initial-actor-jitter-safe-x-min",
            "-0.755",
            "--initial-actor-jitter-safe-x-max",
            "-0.315",
            "--initial-actor-jitter-safe-y-min",
            "-0.245",
            "--initial-actor-jitter-safe-y-max",
            "-0.135",
            "--initial-actor-jitter-center-x-min",
            "-0.700",
            "--initial-actor-jitter-center-x-max",
            "-0.350",
            "--initial-actor-jitter-center-y-min",
            "-0.235",
            "--initial-actor-jitter-center-y-max",
            "-0.145",
            "--enable-grasp-restage-fallback",
            "--grasp-restage-after-attempts",
            "3",
        ]
        case_record["command"] = command
        stdout_path = case_dir / "runner_stdout.txt"
        stderr_path = case_dir / "runner_stderr.txt"
        started = time.perf_counter()
        with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open("w", encoding="utf-8") as stderr:
            completed = subprocess.run(command, cwd=str(ROOT), stdout=stdout, stderr=stderr, check=False)
        elapsed = time.perf_counter() - started
        result = classify_case(case_dir, seed, elapsed, completed.returncode)
        result["index"] = index
        result["stdout_path"] = str(stdout_path)
        result["stderr_path"] = str(stderr_path)
        cases[-1] = result
        write_json(progress_path, {"status": "running", "cases": cases, "aggregate": aggregate(cases)})
        write_json(aggregate_path, aggregate(cases))
    write_json(progress_path, {"status": "finished", "cases": cases, "aggregate": aggregate(cases)})
    write_json(aggregate_path, aggregate(cases))
    print(json.dumps(aggregate(cases), ensure_ascii=False, indent=2))
    return 0 if all(case.get("success") for case in cases) else 1


if __name__ == "__main__":
    raise SystemExit(main())
