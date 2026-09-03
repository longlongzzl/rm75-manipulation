#!/usr/bin/env python3
"""Measure grasp endpoint feasibility across collision-model jaw openings."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from rm75_app.core.frames import robot_base_world_pose
from rm75_app.orchestration.multi_object_executor import load_task_scene
from rm75_app.pickplace.atom_task_builder import AtomTaskBuilderConfig, FixedSceneAtomTaskBuilder
from rm75_app.pickplace.coordinator import _approach_offset_candidates
from rm75_app.planning.backends.curobo2 import Curobo2Backend, Curobo2BackendConfig
from rm75_app.planning.gripper_collision import gripper_link_transforms
from rm75_app.tasks.manipulation_plan import load_plan


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--joint-q",
        type=float,
        nargs="+",
        default=(0.0, 0.4, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80),
    )
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
            )
        )(atom, scene)
        grasps = task.grasp_candidates
        pregrasps = _approach_offset_candidates(
            grasps, abs(float(task.grasp_approach_offset))
        )
        controller = backend._ensure_gripper_sphere_controller()
        urdf_path = next(iter(controller.reference_transforms.values()), None)
        del urdf_path
        robot_yaml = Path(backend.config.robot_config)
        import yaml

        source = yaml.safe_load(robot_yaml.read_text(encoding="utf-8"))
        actual_urdf = Path(source["robot_cfg"]["kinematics"]["urdf_path"])
        records = []
        for q in args.joint_q:
            state_name = f"probe_{float(q):.3f}"
            transforms = gripper_link_transforms(actual_urdf, float(q))
            controller.state_transforms[state_name] = transforms
            backend._gripper_collision_state = state_name
            backend._pose_ik_cache.clear()
            backend._pose_ik_metrics.clear()
            backend._apply_gripper_collision_state()
            backend.prepare_pose_candidates_coarse(
                pregrasps,
                task.scene,
                tool_frame=task.tool_frame,
                ignore_object_name=task.object_name,
            )
            pregrasp_ids = backend.feasible_pose_candidate_ids(pregrasps)
            backend.prepare_pose_candidates_coarse(
                grasps,
                task.scene,
                tool_frame=task.tool_frame,
                ignore_object_name=task.object_name,
            )
            grasp_ids = backend.feasible_pose_candidate_ids(grasps)
            complete_ids = [
                item.candidate_id
                for item in grasps
                if item.candidate_id in grasp_ids
                and f"pregrasp:{item.candidate_id}" in pregrasp_ids
            ]
            left = transforms["left_pad"][:3, 3]
            right = transforms["right_pad"][:3, 3]
            records.append(
                {
                    "joint_q": float(q),
                    "pad_center_distance_m": float(np.linalg.norm(left - right)),
                    "pregrasp_feasible_count": len(pregrasp_ids),
                    "grasp_feasible_count": len(grasp_ids),
                    "complete_count": len(complete_ids),
                    "complete_candidate_ids": complete_ids,
                }
            )
        payload = {
            "object_id": atom.object_id,
            "candidate_count": len(grasps),
            "records": records,
        }
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0
    finally:
        backend.close()


if __name__ == "__main__":
    raise SystemExit(main())
