#!/usr/bin/env python3
"""Compare ManiSkill settling from several tabletop release clearances."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from rm75_app.execution.maniskill_scene import ensure_pick_env_registered
from rm75_app.runtime.curobo2_sim_replay import DEFAULT_EXTRA_MANISKILL_ROOT
from rm75_app.tasks.manipulation_plan import ManipulationPlan
from rm75_app.validation.maniskill_gate import ManiSkillJointAdapter, write_scene_spec


def _rotation_error_deg(reference: np.ndarray, observed: np.ndarray) -> float:
    relative = reference[:3, :3].T @ observed[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.rad2deg(np.arccos(cosine)))


def _matrix(actor) -> np.ndarray:
    value = actor.pose.to_transformation_matrix()
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float64).reshape(-1, 4, 4)[0]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--atom-id", required=True)
    parser.add_argument("--heights-mm", type=float, nargs="+", default=(2.0, 5.0))
    parser.add_argument("--seeds", type=int, nargs="+", default=(0,))
    parser.add_argument("--settle-steps", type=int, default=120)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    plan = ManipulationPlan.from_dict(
        json.loads(args.plan.expanduser().read_text(encoding="utf-8"))
    )
    atom = next(item for item in plan.atoms if item.atom_id == args.atom_id)
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    scene_spec, _ = write_scene_spec(plan, output / "maniskill_scene.json")

    ensure_pick_env_registered("Two_finger_PickJiaobang-v1", DEFAULT_EXTRA_MANISKILL_ROOT)
    import gymnasium as gym
    from mani_skill.utils.structs.pose import Pose
    from transforms3d.quaternions import mat2quat
    import rm75_app.simulation.maniskill_multi_object_env  # noqa: F401

    target = np.asarray(atom.target_pose, dtype=np.float64)
    results = []
    for seed in args.seeds:
      for height_mm in args.heights_mm:
        env = gym.make(
            "RM75-MultiObjectTask-v1",
            scene_spec_file=str(scene_spec),
            robot_uids="RM75",
            obs_mode="none",
            control_mode="pd_joint_pos",
            render_mode="rgb_array",
            max_episode_steps=max(1000, args.settle_steps + 10),
        )
        try:
            env.reset(seed=int(seed))
            adapter = ManiSkillJointAdapter(env)
            actor = env.unwrapped.task_actors[atom.object_id]
            release = target.copy()
            release[2, 3] += float(height_mm) * 0.001
            actor.set_pose(
                Pose.create_from_pq(
                    p=release[:3, 3].astype(np.float32),
                    q=mat2quat(release[:3, :3]).astype(np.float32),
                    device=env.unwrapped.device,
                )
            )
            zero = np.zeros(3, dtype=np.float32)
            actor.set_linear_velocity(zero)
            actor.set_angular_velocity(zero)
            action = adapter.hold_action()
            for _ in range(int(args.settle_steps)):
                env.step(action)
            observed = _matrix(actor)
            results.append(
                {
                    "seed": int(seed),
                    "release_height_mm": float(height_mm),
                    "xy_drift_mm": float(
                        1000.0 * np.linalg.norm(observed[:2, 3] - target[:2, 3])
                    ),
                    "position_error_mm": float(
                        1000.0 * np.linalg.norm(observed[:3, 3] - target[:3, 3])
                    ),
                    "orientation_error_deg": _rotation_error_deg(target, observed),
                    "final_xyz_m": observed[:3, 3].tolist(),
                }
            )
        finally:
            env.close()
    result_file = output / "release_settle_result.json"
    result_file.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(results, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
