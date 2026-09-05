#!/usr/bin/env python3
"""Local simulator-only arm hold audit; no real robot API or planner."""
import argparse
from dataclasses import asdict
import inspect
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rm75_app.validation.maniskill_gate import ManiSkillJointAdapter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-spec", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    import gymnasium as gym
    import rm75_app.simulation.maniskill_multi_object_env  # noqa: F401
    env = gym.make("RM75-MultiObjectTask-v1", scene_spec_file=str(args.scene_spec.resolve()),
                   robot_uids="RM75", obs_mode="none", control_mode="pd_joint_pos",
                   render_mode="rgb_array", max_episode_steps=10000)
    report = {"state": "NEEDS_REVIEW", "physical_commands": 0,
              "scene_spec": str(args.scene_spec), "phases": []}
    try:
        env.reset(seed=0)
        adapter = ManiSkillJointAdapter(env)
        adapter.audit_joint_limits = True
        robot = env.unwrapped.agent.robot
        agent = env.unwrapped.agent
        report["agent_source"] = inspect.getfile(type(agent))
        report["urdf_path"] = str(agent.urdf_path)
        report["controllers"] = {name: asdict(controller.config)
                                 for name, controller in agent.controller.controllers.items()}
        report["link_gravity_disabled"] = {link.name: str(link.disable_gravity) for link in robot.links}
        target = adapter.current_arm_qpos().copy()
        report["arm_target"] = target.tolist()
        report["joint_names"] = list(adapter.arm_names)
        for phase, value in [("open_initial", -1.0), ("closed", 1.0), ("open_final", -1.0)]:
            rows = []
            for tick in range(60):
                adapter.step_and_render(adapter.compose_action(target, value), tag=phase)
                rows.append({"time_s": (tick + 1) / float(env.unwrapped.control_freq),
                             "arm_qpos": adapter.current_arm_qpos().tolist(),
                             "gripper_qpos": adapter.current_gripper_qpos(),
                             "max_arm_error_rad": float(np.max(np.abs(adapter.current_arm_qpos() - target)))})
            report["phases"].append({"phase": phase, "samples": rows,
                                     "final_error_rad": rows[-1]["max_arm_error_rad"],
                                     "max_error_rad": max(r["max_arm_error_rad"] for r in rows)})
        report["limit_and_contact_samples"] = adapter.joint_limit_samples
    finally:
        env.close()
    (args.output_dir / "result.json").write_text(json.dumps(report, indent=2, default=str))
    print(args.output_dir / "result.json")


if __name__ == "__main__":
    main()
