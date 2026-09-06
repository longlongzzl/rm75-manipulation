#!/usr/bin/env python3
"""Read-only substep instrumentation of an ORIGINAL frozen ManiSkill program.

No retiming, dwell, controller or collision changes. Shadow readiness cannot
affect execution. Raw evidence is gzip JSONL to keep per-contact points intact.
"""
import argparse
from dataclasses import asdict
import gzip
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rm75_app.execution.trajectory_executor import ManiSkillTrajectoryExecutor
from rm75_app.runtime.curobo2_sim_replay import load_replay_events
from rm75_app.tasks.manipulation_plan import ManipulationPlan
from rm75_app.validation.maniskill_gate import joint_limit_excess, run_maniskill_gate


def array(value):
    return np.asarray(value.detach().cpu().numpy() if hasattr(value, "detach") else value, dtype=float)


def body_state(body):
    return {"pose": array(body.pose.to_transformation_matrix()).reshape(4, 4).tolist(),
            "linear_velocity_m_s": array(body.get_linear_velocity()).reshape(3).tolist(),
            "angular_velocity_rad_s": array(body.get_angular_velocity()).reshape(3).tolist()}


def shadow_readiness(sample):
    """No invented safe thresholds; known strict-limit violation is not-ready."""
    if sample is None:
        return {"state": "unknown", "reasons": ["no_observation"]}
    reasons = []
    if max(sample["limit_excess_rad"], default=0) > 0:
        reasons.append("joint_limit_excursion")
    return {"state": "not-ready" if reasons else "unknown",
            "reasons": reasons + ["release_motion_contact_thresholds_not_calibrated"],
            "frame_id": sample["frame_id"], "simulation_time_s": sample["simulation_time_s"],
            "shadow_only": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiled-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    manifest = args.compiled_dir / "program/execution.json"
    frozen_path = args.compiled_dir / "frozen_inputs.json"
    frozen = json.loads(frozen_path.read_text())
    plan = ManipulationPlan.from_dict(frozen["plan"])
    atoms = {a.atom_id: a for a in plan.atoms}
    events = load_replay_events(manifest)
    sequence = []
    active = None
    for event in events:
        if event["type"] == "atom_start":
            active = event["atom_id"]
        elif event["type"] == "trajectory":
            sequence.append((active, event["stage"], event["trajectory"]))
    args.output_dir.mkdir(parents=True, exist_ok=False)
    paths = [manifest, frozen_path, Path(plan.scene_file)]
    paths += sorted(manifest.parent.glob("*.npz"))
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    report = {"state": "NEEDS_REVIEW", "original_program": True, "physical_commands": 0,
              "input_hashes_before": hashes, "shadow_readiness": [], "release_boundaries": [],
              "trajectory_dt": [{"atom_id": a, "stage": s, "dt": np.asarray(t.dt).tolist(),
                                  "points": len(t.positions)} for a, s, t in sequence],
              "limits": ["Read-only shadow; no calibrated readiness pass",
                         "Pad-origin distance is aperture proxy, not calibrated inner surface gap",
                         "Contact impulse direction retained as engine pair convention, not inferred force direction"]}
    context = {"active": False, "atom_id": None, "phase": "initial", "frame": 0,
               "command_frame": 0, "latest": None, "trajectory_index": 0}
    from rm75_app.simulation.maniskill_multi_object_env import RM75MultiObjectTaskEnv
    original_before = RM75MultiObjectTaskEnv._before_control_step
    original_after = RM75MultiObjectTaskEnv._after_simulation_step
    original_trajectory = ManiSkillTrajectoryExecutor.execute_trajectory
    original_gripper = ManiSkillTrajectoryExecutor.set_gripper

    with gzip.open(args.output_dir / "physics_substeps.jsonl.gz", "wt", encoding="utf-8") as stream:
        def capture(env):
            # GPU buffers are fetched after control steps in BaseEnv: refusing
            # that backend prevents falsely recording stale states as substeps.
            if env.gpu_sim_enabled:
                raise RuntimeError("substep audit requires CPU physics with immediate state access")
            robot = env.agent.robot
            names = [j.get_name() for j in robot.get_active_joints()]
            q = array(robot.get_qpos()).reshape(-1)
            qdot = array(robot.get_qvel()).reshape(-1)
            limits = array(robot.get_qlimits()).reshape(-1, 2)
            arm_indices = [names.index(f"joint_{i}") for i in range(1, 8)]
            target = array(env.agent.controller.controllers["arm"]._target_qpos).reshape(-1)
            objects = {name: body_state(actor) for name, actor in env.task_actors.items()}
            contacts = []
            for contact in env.scene.get_contacts():
                contacts.append({"body_a": contact.bodies[0].entity.name,
                                 "body_b": contact.bodies[1].entity.name,
                                 "points": [{"position_world_m": array(p.position).tolist(),
                                             "normal_world": array(p.normal).tolist(),
                                             "impulse_ns": array(p.impulse).tolist(),
                                             "separation_m": float(p.separation)} for p in contact.points]})
            left, right = (robot.links_map[name] for name in ("left_pad", "right_pad"))
            l, r = body_state(left), body_state(right)
            delta = np.array(l["pose"])[:3, 3] - np.array(r["pose"])[:3, 3]
            distance = float(np.linalg.norm(delta))
            relative_v = np.array(l["linear_velocity_m_s"]) - r["linear_velocity_m_s"]
            sample = {"frame_id": context["frame"], "command_frame_id": context["command_frame"],
                      "simulation_time_s": context["frame"] / float(env.sim_freq),
                      "coordinate_frame": "ManiSkill world", "atom_id": context["atom_id"],
                      "phase": context["phase"], "q_rad": q.tolist(), "qdot_rad_s": qdot.tolist(),
                      "arm_command_rad": target.tolist(),
                      "arm_tracking_error_rad": (target-q[arm_indices]).tolist(),
                      "limit_excess_rad": joint_limit_excess(q, limits).tolist(),
                      "tcp": body_state(robot.links_map["gripper_tcp"]), "objects": objects,
                      "pad_origin_distance_m": distance,
                      "pad_origin_distance_rate_m_s": float(np.dot(delta, relative_v)/distance) if distance else None,
                      "contacts": contacts}
            if "physics_dt_s" not in report:
                report.update(physics_dt_s=1/float(env.sim_freq), control_dt_s=1/float(env.control_freq),
                              substeps_per_control=int(env._sim_steps_per_control),
                              joint_names=names, joint_limits_rad=limits.tolist(),
                              physics_backend="cpu", time_origin="first execution control step; no wall time")
            stream.write(json.dumps(sample, allow_nan=False) + "\n")
            context["latest"] = sample

        def before(env):
            original_before(env)
            if context["active"]:
                context["command_frame"] += 1

        def after(env):
            original_after(env)
            if context["active"]:
                context["frame"] += 1
                capture(env)

        def trajectory(executor, stage, path):
            index = context["trajectory_index"]
            atom_id, expected_stage, expected = sequence[index]
            if stage != expected_stage or not np.array_equal(path.positions, expected.positions) or not np.array_equal(path.dt, expected.dt):
                raise ValueError("original trajectory mismatch")
            context.update(active=True, atom_id=atom_id, phase=stage, trajectory_index=index+1)
            try:
                return original_trajectory(executor, stage, path)
            finally:
                context["phase"] = "checkpoint"

        def gripper(executor, closed):
            context["phase"] = "gripper_close" if closed else "gripper_open"
            if not closed:
                observation = context["latest"]
                report["shadow_readiness"].append({"atom_id": context["atom_id"],
                                                   **shadow_readiness(observation)})
                report["release_boundaries"].append({"atom_id": context["atom_id"],
                                                     "boundary": "before_open", "observation": observation})
            try:
                result = original_gripper(executor, closed)
                if not closed:
                    report["release_boundaries"].append({"atom_id": context["atom_id"],
                                                         "boundary": "after_open", "observation": context["latest"]})
                return result
            finally:
                context["phase"] = "checkpoint"

        try:
            with patch.object(RM75MultiObjectTaskEnv, "_before_control_step", before), \
                 patch.object(RM75MultiObjectTaskEnv, "_after_simulation_step", after), \
                 patch.object(ManiSkillTrajectoryExecutor, "execute_trajectory", trajectory), \
                 patch.object(ManiSkillTrajectoryExecutor, "set_gripper", gripper):
                result = run_maniskill_gate(plan, manifest, args.output_dir, strict_timed_replay=True)
            report["task_result"] = asdict(result)
            report["task_success"] = result.passed
        except Exception as exc:
            report["exception"] = f"{type(exc).__name__}: {exc}"
    report["physics_frames"] = context["frame"]
    report["control_frames"] = context["command_frame"]
    report["substep_coverage_complete"] = context["frame"] == context["command_frame"] * report.get("substeps_per_control", -1)
    report["input_hashes_unchanged"] = all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (args.output_dir / "evidence_summary.json").write_text(json.dumps(report, indent=2, default=str))
    return 0 if report.get("task_success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
