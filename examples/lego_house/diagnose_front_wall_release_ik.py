from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import sapien

import plan_multi_wall_path_200step as pm
from curobo_rm75_planner import RM75CuRoboPlanner, RM75CuRoboPlannerConfig
from plan_single_wall_path_20s import _candidate_tcp_for_edge_grasp
from record_realman_edge_grasp_open_cube import (
    PLATE_SIZE,
    RM75_HOME,
    _actor_pose,
    _initialize_staged_open_cube,
    _make_env,
    _pose_to_report,
    _set_robot_qpos,
    _tilt_actor_pose_about_local_axis,
)


def _ns(**kwargs: Any) -> argparse.Namespace:
    return argparse.Namespace(**kwargs)


def _json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _json_ready(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_json_ready(v) for v in value]
    if isinstance(value, tuple):
        return [_json_ready(v) for v in value]
    if isinstance(value, np.ndarray):
        return _json_ready(value.tolist())
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def _ik_score(report: dict[str, Any]) -> float:
    debug = dict(report.get("debug") or {})
    pos = float(debug.get("position_error", 1e6) or 1e6)
    rot = float(debug.get("rotation_error", 1e6) or 1e6)
    return pos + 0.02 * rot


def _make_release_candidates(target_pose: sapien.Pose, tilt_deg: float) -> list[tuple[str, sapien.Pose]]:
    if abs(float(tilt_deg)) <= 1e-6:
        return [("exact_target", target_pose)]
    local_bottom_center = np.asarray([0.0, -PLATE_SIZE / 2.0, 0.0], dtype=np.float32)
    local_bottom_axis = np.asarray([1.0, 0.0, 0.0], dtype=np.float32)
    _, q = pm._pose_arrays(target_pose)
    target_rotation = pm.quat2mat(q).astype(np.float32)
    face_normal = pm._normalize_vector(target_rotation @ np.asarray([0.0, 0.0, 1.0], dtype=np.float32))
    candidates: list[tuple[str, sapien.Pose]] = []
    for lift_mm in [0.0, 1.0, 2.0, 4.0, 6.0, 8.0]:
        for normal_bias_mm in [-6.0, -3.0, -1.5, 0.0, 1.5, 3.0, 6.0]:
            world_offset = (
                np.asarray([0.0, 0.0, 0.006 + lift_mm / 1000.0], dtype=np.float32)
                + face_normal * (normal_bias_mm / 1000.0)
            )
            candidates.append(
                (
                    f"tilt_{tilt_deg:+.0f}_lift_{lift_mm:.1f}_normal_{normal_bias_mm:+.1f}",
                    _tilt_actor_pose_about_local_axis(
                        target_pose,
                        local_pivot=local_bottom_center,
                        local_axis=local_bottom_axis,
                        angle_deg=float(tilt_deg),
                        world_offset=world_offset,
                    ),
                )
            )
    return candidates


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--seed", type=int, default=3125)
    parser.add_argument("--ik-seeds", type=int, default=128)
    parser.add_argument("--max-grasp-candidates", type=int, default=96)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--robot-cfg", default=str(Path(__file__).resolve().parent / "curobo_rm75_config" / "rm75.yml"))
    parser.add_argument("--curobo-root", default=r"C:\Users\Administrator\AppData\Local\Temp\curobo-v078")
    parser.add_argument("--ik-position-threshold", type=float, default=0.008)
    parser.add_argument("--ik-rotation-threshold", type=float, default=0.12)
    args = parser.parse_args()

    started = time.perf_counter()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    env = _make_env(False)
    env.reset()
    base_env = env.unwrapped
    _set_robot_qpos(base_env, RM75_HOME, gripper_open=True)
    targets, locked, fixtures = _initialize_staged_open_cube(base_env)
    jitter_args = _ns(
        initial_actor_jitter_xy=0.05,
        initial_actor_jitter_seed=int(args.seed),
        initial_actor_jitter_roles="right_wall,back_wall,left_wall,front_wall",
        initial_actor_jitter_min_start_distance=0.045,
        initial_actor_jitter_min_target_distance=0.025,
        initial_actor_jitter_max_sample_attempts=300,
        initial_actor_jitter_safe_x_min=-0.755,
        initial_actor_jitter_safe_x_max=-0.315,
        initial_actor_jitter_safe_y_min=-0.245,
        initial_actor_jitter_safe_y_max=-0.135,
        initial_actor_jitter_center_x_min=-0.700,
        initial_actor_jitter_center_x_max=-0.350,
        initial_actor_jitter_center_y_min=-0.235,
        initial_actor_jitter_center_y_max=-0.145,
    )
    jitter_report = pm._apply_initial_actor_jitter(
        locked=locked,
        targets=targets,
        fixtures=fixtures,
        args=jitter_args,
        log=lambda _message: None,
    )
    loaded_completed_roles = pm._load_assembly_state(
        path=str(args.state),
        base_env=base_env,
        locked=locked,
        log=lambda _message: None,
        restore_robot_qpos=True,
    )
    restored_connections = pm._restore_loaded_magnetic_connections(
        path=str(args.state),
        base_env=base_env,
        locked=locked,
        log=lambda _message: None,
    )

    planner = RM75CuRoboPlanner(
        RM75CuRoboPlannerConfig(
            curobo_root=Path(args.curobo_root),
            robot_cfg_path=Path(args.robot_cfg),
            position_threshold=float(args.ik_position_threshold),
            rotation_threshold=float(args.ik_rotation_threshold),
            num_ik_seeds=int(args.ik_seeds),
            collision_activation_distance=0.018,
            build_motion_gen=True,
        )
    )

    candidate_args = _ns(
        max_grasp_candidates=int(args.max_grasp_candidates),
        grasp_candidate_start_index=0,
        grasp_candidate_start_indices="",
        square_wall_grasp_edge_axis="any",
        prefer_center_wall_grasp=True,
        max_center_wall_grasp_candidates=24,
        wall_grasp_center_offset=0.004,
        wall_grasp_center_axis_keep_radius=0.012,
        wall_grasp_diversify_pool_size=512,
        wall_grasp_prior_mode="mined_success_v1",
        wall_grasp_prior_max_candidates=0,
        enable_center_grasp_yaw_candidates=False,
        wall_grasp_extra_tilt_degs="-60,-45,-30,-20,-15,0,15,20,30,45,60",
        wall_grasp_extra_tilt_max_abs_deg=60.0,
    )
    grasp_candidates = pm._grasp_candidates_for_role(candidate_args, "front_wall")
    actor = locked["front_wall"].actor
    target_pose = targets["front_wall"]
    start_q = pm._current_q(base_env)
    exclude_roles = {"floor", "front_wall"}

    tasks: list[dict[str, Any]] = []
    for grasp_index, grasp_candidate in enumerate(grasp_candidates):
        tilt_deg = float(getattr(grasp_candidate, "approach_tilt_deg", 0.0))
        grasp_tcp = _candidate_tcp_for_edge_grasp(actor, grasp_candidate)
        actor_to_tcp = _actor_pose(actor).inv() * grasp_tcp
        for release_label, release_pose in _make_release_candidates(target_pose, tilt_deg):
            target_tcp = release_pose * actor_to_tcp
            tasks.append({
                "grasp_index": int(grasp_index),
                "grasp_label": str(getattr(grasp_candidate, "label", "")),
                "tilt_deg": float(tilt_deg),
                "release_label": release_label,
                "target_tcp": _pose_to_report(target_tcp),
                "release_actor_pose": _pose_to_report(release_pose),
                "pose": target_tcp,
            })

    obstacles = pm._world_obstacles_for_stage(
        base_env,
        locked,
        fixtures,
        exclude_role=None,
        exclude_roles=exclude_roles,
    )
    planner.set_world_from_obstacles(cuboids=obstacles)
    records: list[dict[str, Any]] = []
    batch_size = max(int(args.batch_size), 1)
    for batch_start in range(0, len(tasks), batch_size):
        batch = tasks[batch_start: batch_start + batch_size]
        results = planner.solve_batch_start_goal_ik(
            [start_q for _ in batch],
            [pm._world_to_robot_base(base_env, item["pose"]) for item in batch],
            num_seeds=int(args.ik_seeds),
        )
        for task, result in zip(batch, results):
            debug = getattr(result, "debug", {}) or {}
            record = {k: v for k, v in task.items() if k != "pose"}
            record["direct_place_ik"] = {
                "success": bool(getattr(result, "success", False)),
                "status": getattr(result, "status", None),
                "solve_time": float(getattr(result, "solve_time", 0.0) or 0.0),
                "ik_time": float(getattr(result, "ik_time", 0.0) or 0.0),
                "obstacle_count": len(obstacles),
                "debug": debug,
            }
            goal_joint = getattr(result, "goal_joint", None)
            record["direct_q"] = None if goal_joint is None else np.asarray(goal_joint, dtype=np.float32).reshape(-1)[:7].tolist()
            records.append(record)

    direct_successes = sorted(
        [item for item in records if bool(item["direct_place_ik"].get("success"))],
        key=lambda item: _ik_score(item["direct_place_ik"]),
    )
    direct_failures = [item for item in records if not bool(item["direct_place_ik"].get("success"))]
    best_direct_failures = sorted(direct_failures, key=lambda item: _ik_score(item["direct_place_ik"]))[:20]
    payload = {
        "state": str(args.state),
        "seed": int(args.seed),
        "elapsed_sec": time.perf_counter() - started,
        "loaded_completed_roles": loaded_completed_roles,
        "restored_connection_count": len(restored_connections),
        "initial_actor_jitter": jitter_report,
        "front_wall_start_pose": _pose_to_report(_actor_pose(actor)),
        "front_wall_target_pose": _pose_to_report(target_pose),
        "collision_obstacle_policy": {
            "exclude_roles": sorted(exclude_roles),
            "included_completed_roles": ["right_wall", "back_wall", "left_wall"],
            "fixtures_included": True,
        },
        "checked_pairs": int(len(records)),
        "direct_place_success_count": len(direct_successes),
        "direct_successes_top": direct_successes[:20],
        "best_direct_failures": best_direct_failures,
    }
    out_path.write_text(json.dumps(_json_ready(payload), ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(_json_ready({
        "out": str(out_path),
        "checked_pairs": len(records),
        "direct_place_success_count": len(direct_successes),
        "elapsed_sec": payload["elapsed_sec"],
    }), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
