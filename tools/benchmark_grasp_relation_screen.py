#!/usr/bin/env python3
"""Replay frozen plans through Curobo2 relation screening and save JSONL.

The timing stops immediately before the segmented MotionGen chain.  It is
therefore suitable for TASK 001's candidate-build plus relation-screen target,
without commanding a robot, recording trajectories, or running ManiSkill.

This utility measures the checkout it is invoked from.  A comparison against a
legacy implementation still requires a checkout (or explicit implementation)
of that legacy screener; it does not infer one from timing data.
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


class _UnusedExecutor:
    """The screen-only coordinator never calls this executor."""

    def execute_trajectory(self, stage: str, trajectory: object) -> None:
        raise AssertionError(f"relation-screen benchmark unexpectedly executed {stage!r}")

    def set_gripper(self, closed: bool) -> None:
        raise AssertionError(f"relation-screen benchmark unexpectedly set gripper={closed}")


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
    coordinator: _ScreenOnlyCoordinator,
    plan_path: Path,
    atom: ManipulationAtom,
    scene_path: Path,
    repetition: int,
    scene_metadata: Mapping[str, Any],
) -> dict[str, Any]:
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
    result = coordinator.run(task)
    _synchronize(backend)
    screen_wall_time_s = time.perf_counter() - screen_started
    diagnostic = dict(result.diagnostics.get("relation_screen") or {})
    return {
        "plan": str(plan_path),
        "scene": str(scene_path),
        "atom_id": atom.atom_id,
        "object_id": atom.object_id,
        "repetition": repetition,
        "relation_found": bool(result.success and diagnostic.get("complete_relation_count", 0)),
        "failure_stage": result.failure_stage,
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
        "place_candidate_count": diagnostic.get("place_candidate_count"),
        "complete_relation_count": diagnostic.get("complete_relation_count"),
        **dict(scene_metadata),
        **base_metadata,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, action="append", required=True, help="Frozen manipulation_plan.json; repeat for multiple plans.")
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--output-jsonl", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--coarse-ik-batch-size", type=int, default=64)
    parser.add_argument("--coarse-ik-num-seeds", type=int, default=32)
    parser.add_argument("--num-ik-seeds", type=int, default=32)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.repetitions < 2:
        raise ValueError("--repetitions must be at least 2 so a warm distribution exists")
    output = args.output_jsonl.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    plans = [path.expanduser().resolve() for path in args.plan]
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
        coordinator = _ScreenOnlyCoordinator(backend, _UnusedExecutor())
        rows: list[dict[str, Any]] = []
        for plan_path in plans:
            plan = load_plan(plan_path)
            scene_path = Path(plan.scene_file).expanduser().resolve()
            scene_metadata = {
                "plan_id": plan.plan_id,
                "scene_revision": load_task_scene(str(scene_path)).backend_revision,
            }
            for atom in plan.atoms:
                for repetition in range(args.repetitions):
                    row = _run_one(
                        backend=backend,
                        coordinator=coordinator,
                        plan_path=plan_path,
                        atom=atom,
                        scene_path=scene_path,
                        repetition=repetition,
                        scene_metadata=scene_metadata,
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
        "environment": environment,
        "note": "Screen-only: no MotionGen, trajectory execution, ManiSkill, or robot control was run.",
    }
    summary_path = output.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
