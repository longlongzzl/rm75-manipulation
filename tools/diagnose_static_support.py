#!/usr/bin/env python3
"""Isolated static-release physics diagnostic, NOT a motion replay or real gate.

Initialize object/support at recorded pre-open poses with zero initial velocity.
Keep the robot at environment reset and hold it still. No geometry, friction or
target tuning; all other scene objects remain. This is not an exact continuation
of the original dynamic state and cannot validate robot execution.
"""
import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rm75_app.execution.maniskill_task_bridge import ManiSkillTaskBridge
from rm75_app.validation.maniskill_gate import ManiSkillJointAdapter


def make_static_scene(scene, boundary, object_id, support_id):
    output = copy.deepcopy(scene)
    poses = {object_id: boundary["object_pose"], support_id: boundary["support_pose"]}
    if object_id == support_id or any(pose is None for pose in poses.values()):
        raise ValueError("distinct object/support and recorded poses required")
    ids = [item["object_id"] for item in output["objects"]]
    if any(ids.count(name) != 1 for name in poses):
        raise ValueError("object/support must each exist exactly once")
    for item in output["objects"]:
        if item["object_id"] in poses:
            pose = np.asarray(poses[item["object_id"]], dtype=float)
            if pose.shape != (4, 4) or not np.isfinite(pose).all():
                raise ValueError("invalid recorded pose")
            item["pose"] = pose.tolist()
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-spec", type=Path, required=True)
    parser.add_argument("--replay-result", type=Path, required=True)
    parser.add_argument("--atom-id", required=True)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--support-id", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    replay = json.loads(args.replay_result.read_text())
    matches = [r for r in replay["gripper_trace"]
               if not r["closed"] and r["before"]["atom_id"] == args.atom_id]
    if len(matches) != 1:
        raise ValueError("exactly one recorded opening required")
    scene = make_static_scene(json.loads(args.scene_spec.read_text()), matches[0]["before"],
                              args.object_id, args.support_id)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    spec_path = args.output_dir / "static_scene.json"
    spec_path.write_text(json.dumps(scene, indent=2))
    import gymnasium as gym
    import rm75_app.simulation.maniskill_multi_object_env  # noqa: F401
    env = gym.make("RM75-MultiObjectTask-v1", scene_spec_file=str(spec_path.resolve()),
                   robot_uids="RM75", obs_mode="none", control_mode="pd_joint_pos",
                   render_mode="rgb_array", max_episode_steps=10000)
    rows = []
    try:
        env.reset(seed=0)
        adapter = ManiSkillJointAdapter(env)
        bridge = ManiSkillTaskBridge(env, env.unwrapped.task_actors)
        action = adapter.compose_action(adapter.current_arm_qpos(), -1.0)
        for tick in range(61):
            rows.append({"time_s": tick / float(env.unwrapped.control_freq),
                         "object_pose": bridge.observe_object_pose(args.object_id).tolist(),
                         "support_pose": bridge.observe_object_pose(args.support_id).tolist(),
                         "contacts": [c for c in adapter.robot_contact_pairs(include_nonrobot=True)
                                      if any(args.object_id in c[k] for k in ("body_a", "body_b"))]})
            if tick < 60:
                env.step(action)
    finally:
        env.close()
    result = {"state": "NEEDS_REVIEW", "physical_commands": 0,
              "diagnostic": "static initialization, not trajectory replay",
              "source_scene": str(args.scene_spec), "source_replay": str(args.replay_result),
              "atom_id": args.atom_id, "object_id": args.object_id, "support_id": args.support_id,
              "limitations": ["Initial velocities reset, not original dynamic continuation",
                              "Robot at reset, no grasp/release performed", "Not a SIM gate pass"],
              "samples": rows}
    (args.output_dir / "result.json").write_text(json.dumps(result, indent=2))
    print(args.output_dir / "result.json")


if __name__ == "__main__":
    main()
