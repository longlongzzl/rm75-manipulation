#!/usr/bin/env python3
"""Scan stable pen contact points without relaxing the closing-axis yaw."""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from rm75_app.core.frames import robot_base_world_pose
from rm75_app.orchestration.multi_object_executor import load_task_scene
from rm75_app.pickplace.atom_task_builder import AtomTaskBuilderConfig, FixedSceneAtomTaskBuilder
from rm75_app.pickplace.coordinator import _approach_offset_candidates
from rm75_app.planning.backends.curobo2 import Curobo2Backend, Curobo2BackendConfig
from rm75_app.planning.contracts import Pose, PoseCandidate
from rm75_app.tasks.manipulation_plan import load_plan


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    plan = load_plan(args.plan)
    atom = plan.atoms[0]
    scene = load_task_scene(plan.scene_file)
    scene_payload = json.loads(Path(plan.scene_file).read_text(encoding="utf-8"))
    T_world_base, _ = robot_base_world_pose(scene_payload)
    backend = Curobo2Backend(
        Curobo2BackendConfig(max_batch_size=4, max_goalset=4, num_ik_seeds=64)
    )
    try:
        planner = backend._ensure_planner()
        initial = planner.default_joint_state.position.detach().cpu().numpy().reshape(-1)[:7]
        scene.set_joints(tuple(planner.joint_names), initial)
        task = FixedSceneAtomTaskBuilder(
            config=AtomTaskBuilderConfig(
                robot_base_world_xyz_m=tuple(T_world_base[:3, 3]),
                include_maniskill_workspace_table=True,
                use_continuous_grasp_freedom=False,
            )
        )(atom, scene)
        originals = task.grasp_candidates
        center_by_orientation = {
            (
                round(float(item.metadata["yaw_offset_deg"]), 4),
                round(float(item.metadata["tilt_toward_base_deg"]), 4),
                bool(item.metadata.get("closing_axis_flipped", False)),
            ): item
            for item in originals
            if abs(float(item.metadata.get("axis_shift_m", 0.0))) < 1.0e-8
        }
        shifted = next(
            item
            for item in originals
            if abs(float(item.metadata.get("axis_shift_m", 0.0)) - 0.02) < 1.0e-6
        )
        key = (
            round(float(shifted.metadata["yaw_offset_deg"]), 4),
            round(float(shifted.metadata["tilt_toward_base_deg"]), 4),
            bool(shifted.metadata.get("closing_axis_flipped", False)),
        )
        grasp_axis = np.asarray(shifted.pose.position) - np.asarray(
            center_by_orientation[key].pose.position
        )
        grasp_axis /= np.linalg.norm(grasp_axis)
        old_depth = float(originals[0].metadata["grasp_depth_m"])
        axis_shifts = np.arange(-0.066, 0.0661, 0.004)
        depths = (0.004, 0.008, old_depth, 0.018, 0.024, 0.032, 0.040, 0.050)
        candidates: list[PoseCandidate] = []
        for orientation_index, base in enumerate(center_by_orientation.values()):
            rotation = np.asarray(base.pose.quaternion_wxyz, dtype=np.float64)
            w, x, y, z = rotation
            approach = np.asarray(
                [
                    2.0 * (x * z + y * w),
                    2.0 * (y * z - x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ]
            )
            for depth in depths:
                for axis_shift in axis_shifts:
                    position = (
                        np.asarray(base.pose.position)
                        + grasp_axis * float(axis_shift)
                        - approach * (float(depth) - old_depth)
                    )
                    candidate_id = (
                        f"bi_o{orientation_index:02d}_axis_{axis_shift*1000:+.0f}mm"
                        f"_depth_{depth*1000:.0f}mm"
                    )
                    candidates.append(
                        PoseCandidate(
                            candidate_id,
                            Pose(position, base.pose.quaternion_wxyz),
                            score=base.score,
                            metadata={
                                **dict(base.metadata),
                                "axis_shift_m": float(axis_shift),
                                "grasp_depth_m": float(depth),
                            },
                        )
                    )
        candidates_tuple = tuple(candidates)
        backend.set_gripper_collision_state(False)
        backend.prepare_pose_candidates_coarse(
            candidates_tuple,
            task.scene,
            tool_frame=task.tool_frame,
            ignore_object_name=task.object_name,
        )
        feasible = backend.feasible_pose_candidate_ids(candidates_tuple)
        successful_grasps = tuple(
            item for item in candidates_tuple if item.candidate_id in feasible
        )
        pregrasps = _approach_offset_candidates(
            successful_grasps, abs(float(task.grasp_approach_offset))
        )
        backend.prepare_pose_candidates_coarse(
            pregrasps,
            task.scene,
            tool_frame=task.tool_frame,
            ignore_object_name=task.object_name,
        )
        pregrasp_feasible = backend.feasible_pose_candidate_ids(pregrasps)
        complete = [
            item
            for item in successful_grasps
            if f"pregrasp:{item.candidate_id}" in pregrasp_feasible
        ]
        continuous_inputs = tuple(
            replace(
                item,
                metadata={
                    **dict(item.metadata),
                    "free_rotation_axis_local": "y",
                    "max_closing_axis_world_z": 0.05,
                    "min_downward_approach_cosine": 0.50,
                },
            )
            for item in candidates_tuple
            if abs(float(item.metadata["tilt_toward_base_deg"])) < 1.0e-8
        )
        continuous_resolved = backend.resolve_axis_constrained_pose_candidates(
            continuous_inputs,
            task.scene,
            tool_frame=task.tool_frame,
            ignore_object_names=(task.object_name,),
        )
        continuous_pregrasps = _approach_offset_candidates(
            continuous_resolved, abs(float(task.grasp_approach_offset))
        )
        backend.prepare_pose_candidates_coarse(
            continuous_pregrasps,
            task.scene,
            tool_frame=task.tool_frame,
            ignore_object_name=task.object_name,
        )
        continuous_pregrasp_feasible = backend.feasible_pose_candidate_ids(
            continuous_pregrasps
        )
        continuous_complete = [
            item
            for item in continuous_resolved
            if f"pregrasp:{item.candidate_id}" in continuous_pregrasp_feasible
        ]
        metrics = backend.pose_candidate_metrics(candidates_tuple)
        nearest = sorted(
            (
                {
                    "candidate_id": item.candidate_id,
                    **metrics.get(item.candidate_id, {}),
                }
                for item in candidates_tuple
            ),
            key=lambda item: float(item.get("normalized_pose_gap", 1.0e9)),
        )[:20]
        payload = {
            "candidate_count": len(candidates_tuple),
            "stable_orientation_count": len(center_by_orientation),
            "grasp_feasible_count": len(successful_grasps),
            "pregrasp_feasible_count": len(pregrasp_feasible),
            "complete_count": len(complete),
            "continuous_input_count": len(continuous_inputs),
            "continuous_grasp_feasible_count": len(continuous_resolved),
            "continuous_pregrasp_feasible_count": len(
                continuous_pregrasp_feasible
            ),
            "continuous_complete_count": len(continuous_complete),
            "complete_candidates": [
                {
                    "candidate_id": item.candidate_id,
                    "position": item.pose.position.tolist(),
                    "axis_shift_m": item.metadata["axis_shift_m"],
                    "grasp_depth_m": item.metadata["grasp_depth_m"],
                    "tilt_deg": item.metadata["tilt_toward_base_deg"],
                    "closing_axis_flipped": item.metadata["closing_axis_flipped"],
                }
                for item in complete[:100]
            ],
            "continuous_complete_candidates": [
                {
                    "candidate_id": item.candidate_id,
                    "position": item.pose.position.tolist(),
                    "axis_shift_m": item.metadata["axis_shift_m"],
                    "grasp_depth_m": item.metadata["grasp_depth_m"],
                    "resolved_approach_world_z": item.metadata.get(
                        "resolved_approach_world_z"
                    ),
                }
                for item in continuous_complete[:100]
            ],
            "nearest": nearest,
        }
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0 if complete or continuous_complete else 2
    finally:
        backend.close()


if __name__ == "__main__":
    raise SystemExit(main())
