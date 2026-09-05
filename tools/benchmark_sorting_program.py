#!/usr/bin/env python3
"""Compile a sorting request using cuRobo, without creating any robot sink.

The explicit fixture flag permits planner-default joints for software diagnosis;
such a run is not a measured real-scene benchmark or approval to execute.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark_grasp_relation_screen import _scene_builder, _synchronize
from run_unified_scenario import _write_json
from rm75_app.orchestration.multi_object_executor import load_task_scene
from rm75_app.planning.backends.curobo2 import Curobo2Backend, Curobo2BackendConfig
from rm75_app.planning.contracts import JointConfiguration
from rm75_app.scenarios.pickplace_program import PickPlaceProgramCompiler, TrajectoryCommand
from rm75_app.scenarios.sorting import SortingPlanCompiler
from rm75_app.scenarios.sorting_io import load_sorting_request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--fixture-default-joints", action="store_true")
    args = parser.parse_args()
    # Exclusive run directory prevents silently replacing frozen inputs/results.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    request = load_sorting_request(args.request)
    scene_path = Path(request.scene_file)
    scene = load_task_scene(str(scene_path))
    # The legacy loader intentionally loads objects only, not snapshot joints.
    payload = json.loads(scene_path.read_text())
    if payload.get("joint_positions") is not None:
        observed = JointConfiguration(tuple(payload["joint_names"]), payload["joint_positions"])
        if not observed.names:
            parser.error("snapshot joint names must not be empty")
        scene.set_joints(observed.names, observed.positions)
    plan = SortingPlanCompiler().compile(request, scene)
    if scene.joint_positions is None and not args.fixture_default_joints:
        parser.error("snapshot joints required; fixture diagnostics require --fixture-default-joints")
    config = Curobo2BackendConfig()
    result_summary = {
        "state": "NEEDS_REVIEW", "physical_commands": 0,
        "fixture_only": args.fixture_default_joints or bool(request.metadata.get("example_only")),
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "backend_config": asdict(config), "planned_atoms": len(plan.atoms),
        "limitations": ["Single diagnostic fixture, not the 20 real-snapshot U1 suite.",
                        "No ManiSkill or real execution; no geometric or dynamics success inferred."],
    }
    started = time.perf_counter()
    try:
        with Curobo2Backend(config) as backend:
            native = backend._ensure_planner()
            if args.fixture_default_joints:
                joints = native.default_joint_state.position.detach().cpu().numpy().reshape(-1)[:7]
                scene.set_joints(tuple(native.joint_names), joints)
            _synchronize(backend)
            result_summary["backend_initialization_s"] = time.perf_counter() - started
            frozen = {
                "request": json.loads(args.request.read_text()),
                "scene": scene.as_dict(), "plan": plan.as_dict(),
                "input_hashes": {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (args.request, scene_path, Path(__file__))},
                "joint_source": "planner_default_fixture" if args.fixture_default_joints else "snapshot",
            }
            _write_json(args.output_dir / "frozen_inputs.json", frozen)
            builder, frame_metadata = _scene_builder(scene_path)
            builds = []

            def timed_builder(atom, predicted):
                begin = time.perf_counter()
                task = builder(atom, predicted)
                _synchronize(backend)
                builds.append({"atom_id": atom.atom_id, "source_revision": predicted.revision,
                               "candidate_build_s": time.perf_counter() - begin})
                return task

            compilation = PickPlaceProgramCompiler(backend, timed_builder).compile(plan, scene)
            result_summary.update({
                "success": compilation.success, "failed_atom_id": compilation.failed_atom_id,
                "failure_stage": compilation.failure_stage, "message": compilation.message,
                "diagnostics": compilation.diagnostics, "candidate_builds": builds,
                "frame_metadata": frame_metadata,
            })
            atoms = compilation.program.atoms if compilation.program else compilation.partial_atoms
            result_summary["completed_atoms"] = [
                {"atom_id": a.atom.atom_id, "planning_time_s": a.planning_time_s,
                 "selected_grasp": a.selected_grasp, "selected_place": a.selected_place,
                 "source_revision": a.source_scene_revision, "predicted_revision": a.predicted_scene_revision,
                 "diagnostics": a.diagnostics} for a in atoms]
            if compilation.program:
                program = compilation.program
                result_summary["program_manifest"] = str(program.export(args.output_dir / "program"))
                trajectories = [c for c in program.commands if isinstance(c, TrajectoryCommand)]
                result_summary["stages"] = [{"atom_id": c.atom_id, "stage": c.stage,
                                             "points": len(c.trajectory.positions)} for c in trajectories]
                result_summary["initial_gap_rad"] = float(np.max(np.abs(
                    trajectories[0].trajectory.positions[0] - scene.joint_positions)))
                result_summary["max_inter_stage_gap_rad"] = max((float(np.max(np.abs(
                    a.trajectory.positions[-1] - b.trajectory.positions[0])))
                    for a, b in zip(trajectories, trajectories[1:])), default=0.0)
    except Exception as exc:
        result_summary.update(success=False, failure_stage="benchmark_exception",
                              message=f"{type(exc).__name__}: {exc}")
    result_summary["total_wall_s"] = time.perf_counter() - started
    _write_json(args.output_dir / "summary.json", result_summary)
    return 0 if result_summary.get("success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
