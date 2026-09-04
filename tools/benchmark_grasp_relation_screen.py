#!/usr/bin/env python3
"""Replay frozen plans through Curobo2 relation screening and save JSONL.

The timing stops immediately before the segmented MotionGen chain.  It is
therefore suitable for TASK 001's candidate-build plus relation-screen target,
without commanding a robot, recording trajectories, or running ManiSkill.

With ``--full-chain`` it instead runs production segmented MotionGen planning
through a no-op executor which accepts trajectories and records their stage
names only.  It never commands a robot or runs ManiSkill.  This utility
measures the checkout it is invoked from.  A comparison against a legacy
implementation still requires a checkout (or explicit implementation) of that
legacy screener; it does not infer one from timing data.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Iterable, Mapping

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rm75_app.core.frames import robot_base_world_pose
from rm75_app.orchestration.multi_object_executor import load_task_scene
from rm75_app.pickplace.atom_task_builder import AtomTaskBuilderConfig, FixedSceneAtomTaskBuilder
from rm75_app.pickplace.coordinator import PickPlaceCoordinator, PickPlaceRunResult, PickPlaceTask
from rm75_app.planning.backends.curobo2 import Curobo2Backend, Curobo2BackendConfig
from rm75_app.planning.contracts import PoseCandidate
from rm75_app.tasks.manipulation_plan import ManipulationAtom, load_plan


def _percentile(values: list[float], ratio: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * ratio
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - rank) + ordered[high] * (rank - low)


def _stats(values: Iterable[float]) -> dict[str, float | int | None]:
    data = [float(value) for value in values]
    return {
        "count": len(data),
        "p50": _percentile(data, 0.50),
        "p90": _percentile(data, 0.90),
        "p95": _percentile(data, 0.95),
        "p99": _percentile(data, 0.99),
        "mean": statistics.fmean(data) if data else None,
        "max": max(data) if data else None,
    }


class _ScreenOnlyCoordinator(PickPlaceCoordinator):
    """Use production screening, but stop exactly before segmented MotionGen."""

    def _run_segmented_chain(
        self,
        task: PickPlaceTask,
        relation_grasp_candidates: tuple[PoseCandidate, ...],
        complete_places_by_grasp: Mapping[str, tuple[PoseCandidate, ...]],
        grasp_scores: Mapping[str, float],
        relation_screen: Mapping[str, Any],
    ) -> PickPlaceRunResult:
        del task, grasp_scores
        selected_grasp = relation_grasp_candidates[0]
        selected_places = complete_places_by_grasp[selected_grasp.candidate_id]
        return PickPlaceRunResult(
            True,
            (),
            selected_grasp=selected_grasp.candidate_id,
            selected_place=selected_places[0].candidate_id if selected_places else None,
            diagnostics={"relation_screen": dict(relation_screen)},
        )


class _NoopExecutor:
    """Accept planned trajectories while recording stage names only."""

    def __init__(self) -> None:
        self.stage_names: list[str] = []

    def reset(self) -> None:
        self.stage_names.clear()

    def execute_trajectory(self, stage: str, trajectory: object) -> None:
        del trajectory
        self.stage_names.append(stage)

    def set_gripper(self, closed: bool) -> None:
        del closed


def _scene_builder(scene_path: Path) -> tuple[FixedSceneAtomTaskBuilder, dict[str, Any]]:
    payload = json.loads(scene_path.read_text(encoding="utf-8"))
    world_from_base, source = robot_base_world_pose(payload)
    return (
        FixedSceneAtomTaskBuilder(
            config=AtomTaskBuilderConfig(
                robot_base_world_xyz_m=tuple(world_from_base[:3, 3].tolist()),
                include_maniskill_workspace_table=True,
            )
        ),
        {"robot_base_world_pose_source": source},
    )


def _synchronize(backend: Curobo2Backend) -> None:
    modules = backend._import_modules()
    torch = modules["torch"]
    if bool(torch.cuda.is_available()):
        torch.cuda.synchronize()


def _run_one(
    *,
    backend: Curobo2Backend,
    coordinator: PickPlaceCoordinator,
    executor: _NoopExecutor,
    plan_path: Path,
    atom: ManipulationAtom,
    scene_path: Path,
    repetition: int,
    scene_metadata: Mapping[str, Any],
    relation_screen_mode: str,
) -> dict[str, Any]:
    reset_metrics = getattr(backend, "reset_endpoint_screen_metrics", None)
    if callable(reset_metrics):
        reset_metrics()
    scene = load_task_scene(str(scene_path))
    planner = backend._ensure_planner()
    initial = planner.default_joint_state.position.detach().cpu().numpy().reshape(-1)[:7]
    scene.set_joints(tuple(planner.joint_names), initial)
    builder, base_metadata = _scene_builder(scene_path)
    _synchronize(backend)
    build_started = time.perf_counter()
    task = builder(atom, scene)
    _synchronize(backend)
    candidate_build_wall_time_s = time.perf_counter() - build_started
    _synchronize(backend)
    screen_started = time.perf_counter()
    executor.reset()
    result = coordinator.run(task)
    _synchronize(backend)
    screen_wall_time_s = time.perf_counter() - screen_started
    diagnostic = dict(result.diagnostics.get("relation_screen") or {})
    read_metrics = getattr(backend, "endpoint_screen_metrics", None)
    backend_metrics = read_metrics() if callable(read_metrics) else {}
    manifold = dict(diagnostic.get("place_manifold_resolution") or {})
    return {
        "plan": str(plan_path),
        "scene": str(scene_path),
        "atom_id": atom.atom_id,
        "object_id": atom.object_id,
        "repetition": repetition,
        "relation_screen_mode": relation_screen_mode,
        "relation_found": bool(result.success and diagnostic.get("complete_relation_count", 0)),
        "full_chain_plan_success": bool(result.success),
        "failure_stage": result.failure_stage,
        "executed_stage_names": list(executor.stage_names),
        "segmented_plan_time_s": dict(result.diagnostics.get("timing") or {}).get("segmented_plan_time_s"),
        "candidate_failures": list(result.diagnostics.get("candidate_failures") or ()),
        "grasp_tool_axis_retry_used": bool(
            result.diagnostics.get("grasp_tool_axis_retry_used", False)
        ),
        "grasp_primary_status": result.diagnostics.get("grasp_primary_status"),
        "grasp_tool_axis_retry_status": result.diagnostics.get(
            "grasp_tool_axis_retry_status"
        ),
        "grasp_reverse_fallback_used": bool(
            result.diagnostics.get("grasp_reverse_fallback_used", False)
        ),
        "grasp_reverse_start_gap_rad": result.diagnostics.get(
            "grasp_reverse_start_gap_rad"
        ),
        "grasp_reverse_probe_status": result.diagnostics.get(
            "grasp_reverse_probe_status"
        ),
        "cached_grasp_available": result.diagnostics.get("cached_grasp_available"),
        "cached_grasp_distance_from_pregrasp": result.diagnostics.get(
            "cached_grasp_distance_from_pregrasp"
        ),
        "reverse_probe_trajectory_points": result.diagnostics.get(
            "reverse_probe_trajectory_points"
        ),
        "reversed_start_gap_rad": result.diagnostics.get("reversed_start_gap_rad"),
        "selected_grasp": result.selected_grasp,
        "selected_place": result.selected_place,
        "candidate_build_wall_time_s": candidate_build_wall_time_s,
        "screen_wall_time_s": screen_wall_time_s,
        "candidate_build_time_s": diagnostic.get("candidate_build_time_s"),
        "screen_total_time_s": diagnostic.get("screen_total_time_s"),
        "grasp_family_screen_time_s": diagnostic.get("grasp_family_screen_time_s"),
        "place_family_screen_time_s": diagnostic.get("place_family_screen_time_s"),
        "coarse_ik_call_count": diagnostic.get("coarse_ik_call_count"),
        "coarse_ik_rows_requested": diagnostic.get("coarse_ik_rows_requested"),
        "coarse_ik_rows_padded": diagnostic.get("coarse_ik_rows_padded"),
        "stable_ik_call_count": diagnostic.get("stable_ik_call_count"),
        "search_tier": diagnostic.get("search_tier"),
        "grasp_candidate_count": diagnostic.get("grasp_candidate_count"),
        "declared_place_candidate_count": diagnostic.get("unique_place_candidate_count"),
        "place_candidate_count": diagnostic.get("place_candidate_count"),
        "eligible_place_candidate_count": diagnostic.get("eligible_place_candidate_count"),
        "pose_manifold_input_count": manifold.get("input_count"),
        "pose_manifold_resolved_count": manifold.get("resolved_count"),
        "complete_relation_count": diagnostic.get("complete_relation_count"),
        "backend_endpoint_screen_metrics": backend_metrics,
        **dict(scene_metadata),
        **base_metadata,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, action="append", required=True, help="Frozen manipulation_plan.json; repeat for multiple plans.")
    parser.add_argument(
        "--scene",
        type=Path,
        action="append",
        default=None,
        help="Optional frozen scene override; provide one per --plan.",
    )
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--output-jsonl", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--coarse-ik-batch-size", type=int, default=64)
    parser.add_argument("--coarse-ik-num-seeds", type=int, default=32)
    parser.add_argument("--num-ik-seeds", type=int, default=32)
    parser.add_argument(
        "--relation-screen-mode",
        choices=("eager", "lazy_place", "lazy_place_progressive_preplace"),
        default=None,
        help="Override the coordinator mode; omit to exercise its production default.",
    )
    parser.add_argument(
        "--grasp-fallback-mode",
        choices=("primary_only", "reverse_probe_experimental"),
        default="primary_only",
        help="Use reverse_probe_experimental only for E1B-alt validation.",
    )
    parser.add_argument(
        "--full-chain",
        action="store_true",
        help="Run production segmented MotionGen with a no-op trajectory executor.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.repetitions < 1:
        raise ValueError("--repetitions must be positive")
    output = args.output_jsonl.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    plans = [path.expanduser().resolve() for path in args.plan]
    scene_overrides = (
        [path.expanduser().resolve() for path in args.scene]
        if args.scene is not None
        else None
    )
    if scene_overrides is not None and len(scene_overrides) != len(plans):
        raise ValueError("--scene must be given exactly once for every --plan")
    config = Curobo2BackendConfig(
        device=str(args.device),
        coarse_ik_batch_size=int(args.coarse_ik_batch_size),
        coarse_ik_num_seeds=int(args.coarse_ik_num_seeds),
        num_ik_seeds=int(args.num_ik_seeds),
    )
    construction_started = time.perf_counter()
    with Curobo2Backend(config) as backend:
        planner = backend._ensure_planner()
        _synchronize(backend)
        planner_construction_s = time.perf_counter() - construction_started
        modules = backend._import_modules()
        torch = modules["torch"]
        environment = {
            "device": str(args.device),
            "torch_version": str(torch.__version__),
            "cuda_available": bool(torch.cuda.is_available()),
            "cuda_device_name": str(torch.cuda.get_device_name(0)) if bool(torch.cuda.is_available()) else None,
            "planner_joint_count": len(planner.joint_names),
            "backend_config": asdict(config),
        }
        executor = _NoopExecutor()
        coordinator_class = PickPlaceCoordinator if args.full_chain else _ScreenOnlyCoordinator
        coordinator_kwargs = (
            {} if args.relation_screen_mode is None
            else {"relation_screen_mode": args.relation_screen_mode}
        )
        coordinator = coordinator_class(
            backend,
            executor,
            grasp_fallback_mode=str(args.grasp_fallback_mode),
            **coordinator_kwargs,
        )
        active_relation_screen_mode = coordinator.relation_screen_mode
        rows: list[dict[str, Any]] = []
        for plan_index, plan_path in enumerate(plans):
            plan = load_plan(plan_path)
            scene_path = (
                scene_overrides[plan_index]
                if scene_overrides is not None
                else Path(plan.scene_file).expanduser().resolve()
            )
            scene_metadata = {
                "plan_id": plan.plan_id,
                "scene_revision": load_task_scene(str(scene_path)).backend_revision,
            }
            for atom in plan.atoms:
                for repetition in range(args.repetitions):
                    row = _run_one(
                        backend=backend,
                        coordinator=coordinator,
                        executor=executor,
                        plan_path=plan_path,
                        atom=atom,
                        scene_path=scene_path,
                        repetition=repetition,
                        scene_metadata=scene_metadata,
                        relation_screen_mode=active_relation_screen_mode,
                    )
                    rows.append(row)
                    print(
                        f"{atom.atom_id} rep={repetition:02d} "
                        f"screen={row['screen_wall_time_s']:.3f}s "
                        f"relation={row['relation_found']}",
                        flush=True,
                    )
    with output.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
    warm_rows = [row for row in rows if int(row["repetition"]) > 0]
    def task_summary(task_rows: list[dict[str, Any]]) -> dict[str, Any]:
        warm_task_rows = [row for row in task_rows if int(row["repetition"]) > 0]
        by_kind: dict[str, dict[str, float]] = {}
        for row in warm_task_rows:
            for kind, values in dict(row["backend_endpoint_screen_metrics"]).items():
                aggregate = by_kind.setdefault(str(kind), {"solver_calls": 0.0, "rows_requested": 0.0, "rows_padded": 0.0})
                for key in aggregate:
                    aggregate[key] += float(values.get(key, 0))
        count = len(warm_task_rows)
        if count:
            by_kind = {kind: {key: value / count for key, value in values.items()} for kind, values in by_kind.items()}
        first = task_rows[0]
        return {
            "object_id": first["object_id"],
            "atom_id": first["atom_id"],
            "warm_relation_screen_wall_time_s": _stats([float(row["screen_wall_time_s"]) for row in warm_task_rows]),
            "relation_found_rate": sum(bool(row["relation_found"]) for row in warm_task_rows) / count if count else None,
            "selected_search_tiers": sorted({row["search_tier"] for row in warm_task_rows}),
            "backend_endpoint_screen_metrics_mean": by_kind,
        }

    rows_by_task: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for row in rows:
        key = (
            str(row["relation_screen_mode"]),
            str(row["plan_id"]),
            str(row["atom_id"]),
            str(row["object_id"]),
        )
        rows_by_task.setdefault(key, []).append(row)
    summary = {
        "raw_jsonl": str(output),
        "planner_construction_s": planner_construction_s,
        "plans": [str(path) for path in plans],
        "repetitions_per_atom": args.repetitions,
        "sample_count": len(rows),
        "warm_sample_count": len(warm_rows),
        "cold_first_call_s": _stats([float(row["screen_wall_time_s"]) for row in rows if int(row["repetition"]) == 0]),
        "warm_candidate_build_wall_time_s": _stats([float(row["candidate_build_wall_time_s"]) for row in warm_rows]),
        "warm_relation_screen_wall_time_s": _stats([float(row["screen_wall_time_s"]) for row in warm_rows]),
        "warm_relation_screen_diagnostic_time_s": _stats([float(row["screen_total_time_s"]) for row in warm_rows if row["screen_total_time_s"] is not None]),
        "relation_found_rate": sum(bool(row["relation_found"]) for row in warm_rows) / len(warm_rows) if warm_rows else None,
        "mean_coarse_ik_calls": statistics.fmean(float(row["coarse_ik_call_count"]) for row in warm_rows) if warm_rows else None,
        "mean_coarse_ik_rows_requested": statistics.fmean(float(row["coarse_ik_rows_requested"]) for row in warm_rows) if warm_rows else None,
        "mean_coarse_ik_rows_padded": statistics.fmean(float(row["coarse_ik_rows_padded"]) for row in warm_rows) if warm_rows else None,
        "tasks": {"/".join(key): task_summary(task_rows) for key, task_rows in rows_by_task.items()},
        "environment": environment,
        "note": (
            "Full segmented MotionGen planning used a no-op stage-recording executor; "
            "no robot control or ManiSkill was run."
            if args.full_chain else
            "Screen-only: no MotionGen, trajectory execution, ManiSkill, or robot control was run."
        ),
    }
    summary_path = args.summary_json.expanduser().resolve() if args.summary_json is not None else output.with_suffix(".summary.json")
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
