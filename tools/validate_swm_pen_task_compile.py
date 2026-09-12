"""Compile original pen insertion candidates from a captured native SWM scene.

This is an offline compiler probe, not model inference or motion validation.
Run under tools/run_network_isolated.py, as with all release experiments.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from rm75_app.llm.orchestrator import SceneObject, SceneState, _resolve_single_step
    from rm75_app.orchestration.multi_object_executor import SceneObjectState, TaskSceneState
    from rm75_app.pickplace.atom_task_builder import AtomTaskBuilderConfig, FixedSceneAtomTaskBuilder
    from rm75_app.swm.native_tasks import CompiledNativeTask
    from rm75_app.tasks.manipulation_plan import compile_resolved_steps

    raw = args.capture.read_bytes()
    snapshot = json.loads(raw)['snapshot']
    T_world_base = np.asarray(snapshot['robot']['T_world_base'], dtype=float)
    states = {}
    semantic_objects = {}
    for oid, row in snapshot['objects'].items():
        asset = snapshot['assets'][row['asset_id']]
        pose = T_world_base @ np.asarray(row['measured']['T_world_object'])
        states[oid] = SceneObjectState(oid, asset['native_asset_name'], pose,
                                       movable=not row['fixed'])
        if not asset.get('native_infrastructure'):
            semantic_objects[oid] = SceneObject(oid, asset['native_asset_name'], oid,
                                                1.0, False, pose)
    scene_file = ROOT / 'assets/test_scenes/current_table.json'
    semantic = SceneState(scene_file, semantic_objects, {})
    sources = semantic.find_by_spec('bi')
    targets = semantic.find_by_spec('bitong')
    if len(sources) != 1 or len(targets) != 1:
        raise ValueError('Probe requires unambiguous original pen and holder instances')
    resolved = _resolve_single_step(semantic, dict(operator='inside',
        source_ref=sources[0].object_id, target_ref=targets[0].object_id), 1)
    plan = compile_resolved_steps(plan_id='native_pen_insertion_compile_probe',
        scene_file=scene_file, steps=[dict(source_id=resolved.source_id,
            source_spec='bi', operator=resolved.operator,
            target_object_id=resolved.target_object_id, target_pose=resolved.target_pose,
            primitive=resolved.primitive, place_mode=resolved.place_mode,
            description=resolved.description, warnings=resolved.warnings)])
    builder = FixedSceneAtomTaskBuilder(config=AtomTaskBuilderConfig(
        robot_base_world_transform=T_world_base, include_maniskill_workspace_table=False))
    bridge = CompiledNativeTask('pickplace', plan, TaskSceneState(states), builder)
    requests = bridge.requests()
    grasp = next(requests)
    task = bridge.build_task(grasp, snapshot)
    if set(task.place_candidates_by_grasp) != {g.candidate_id for g in task.grasp_candidates}:
        raise RuntimeError('Original grasp/place pairing is incomplete')
    result = dict(status='passed', domain='offline_original_native_task_compiler',
        capture_sha256=hashlib.sha256(raw).hexdigest(), snapshot_id=snapshot['snapshot_id'],
        source_id=resolved.source_id, target_id=resolved.target_object_id,
        primitive=resolved.primitive, place_mode=resolved.place_mode,
        target_world=np.asarray(resolved.target_pose).tolist(),
        collision_object_count=len(task.scene.objects),
        grasp_candidate_count=len(task.grasp_candidates),
        place_candidate_count=len(task.place_candidates),
        paired_place_counts={key: len(value) for key, value in task.place_candidates_by_grasp.items()},
        solver_feasibility_verified=False, atomic_worker_qualified=False,
        skill_checkpoint_count=0, model_inference_run=False, motion_executed=False,
        hardware_connected=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
