"""Trusted, owned pen runtime for the existing worker, not a legacy episode.

Only the original frozen-world SIM pen task is currently bound. Installation
is not qualification; all other tasks and real hardware remain unavailable.
"""
from __future__ import annotations

import copy
import os
from pathlib import Path
import sys
import time
import uuid

from .integration import RuntimeContext, RuntimeSession


def install_native_factories(profile):
    from . import integration
    options = profile.get('swm', {})
    section = profile.get('pickplace', {})
    if (not isinstance(options, dict) or options.get('enabled') is not True
            or section.get('fixed_scene_format') != 'native_world'
            or not section.get('fixed_scene')):
        return
    existing = integration._FACTORIES.get('pickplace')
    if existing is None:
        integration.register_runtime_factory('pickplace', pen_runtime_context)
    elif existing is not pen_runtime_context:
        raise RuntimeError('Do not replace an installed SWM runtime factory')


def pen_runtime_context(spec, profile, app_root, run_dir, stop, events):
    if spec.get('mode') != 'sim':
        raise PermissionError('Native pen runtime is offline simulation only')
    if spec.get('task') != 'pickplace' or spec.get('parameters') != {'object_name': 'bi'}:
        raise NotImplementedError('SWM_ATOMIC_ADAPTER_REQUIRED: only the original single pen SIM boundary is bound')
    section = profile.get('pickplace', {})
    if section.get('fixed_scene_format') != 'native_world' or not section.get('fixed_scene'):
        raise NotImplementedError('SWM_ATOMIC_ADAPTER_REQUIRED: original frozen-world input required')
    return RuntimeContext(lambda resources: _build_pen_session(
        resources, copy.deepcopy(spec), copy.deepcopy(profile), Path(app_root).resolve(),
        Path(run_dir).resolve(), stop, events))


def _build_pen_session(resources, spec, profile, app_root, run_dir, stop, events):
    # A second installation proves the exact guard before native imports, not
    # merely that some unrelated seccomp filter happens to be active.
    from tools.run_network_isolated import block_network
    block_network()
    stop.check()
    old_argv, old_cwd, old_path = sys.argv[:], Path.cwd(), sys.path[:]
    resources.callback(lambda: setattr(sys, 'argv', old_argv))
    resources.callback(os.chdir, old_cwd)
    resources.callback(lambda: setattr(sys, 'path', old_path))
    # These are administrator-owned machine profile paths, never request fields.
    dependencies = profile.get('swm', {}).get('dependency_site_packages', [])
    if (not isinstance(dependencies, list) or len(dependencies) > 4
            or any(not isinstance(p, str) or not Path(p).is_absolute()
                   or not Path(p).is_dir() for p in dependencies)):
        raise ValueError('SWM dependency paths require existing absolute server directories')
    for path in dependencies:
        if path not in sys.path:
            sys.path.append(path)
    from rm75_app.workcell.legacy import (
        snapshot_root, import_working_entry, build_native_argv, PICKPLACE_WORLD_ENTRY)
    from rm75_app.workcell.native_frozen_world import read_contract
    from rm75_app.workcell.pickplace_curobo_only import source_adapter
    from .native_bootstrap import initialize_frozen_primary
    from .native_registration import register_primary_scene
    from .native_robot_mirror import SapienRobotStatePort
    from .native_body_mirror import NativeBodyMirror, native_pose
    from .native_scene import SapienScenePort, CuroboScenePort
    from .adapters import TransactionalSceneMirror, NativeAtomicBackend
    from .scene import CheckpointSynchronizer
    from .native_execution import NativePrimaryExecutor
    from .native_skills import PickPlaceNativePhases, SharedPrimitiveExecutor
    from .native_audit import CuroboNativeStageAuditor
    from .skills import AtomicSkillRuntime
    from rm75_app.planning.backends.curobo2 import Curobo2Backend, Curobo2BackendConfig
    from rm75_app.planning.contracts import PlanningScene
    from rm75_app.pickplace.coordinator import PickPlaceCoordinator

    output = run_dir / 'swm_native'
    output.mkdir(exist_ok=False)
    root = snapshot_root(app_root)
    fixed = Path(profile['pickplace']['fixed_scene'])
    if not fixed.is_absolute():
        fixed = root / fixed
    contract = read_contract(fixed, 'bi')
    resources.enter_context(source_adapter(root))
    profile['pickplace']['render_mode'] = 'none'
    os.chdir(root)
    direct = import_working_entry(root, 'pickplace', entrypoint=PICKPLACE_WORLD_ENTRY)
    argv = build_native_argv(direct, spec, profile, output, root, frozen_contract=contract)
    sys.argv = [str(root / PICKPLACE_WORLD_ENTRY), *argv]
    args = direct.parse_args()
    args.foundationpose_refine_after_render = False
    primary = initialize_frozen_primary(resources, direct, args, contract, stop=stop, events=events,
        artifact_directory=output / 'initialization_artifacts', capture_builders=True)
    backend = resources.enter_context(Curobo2Backend(Curobo2BackendConfig()))
    backend.update_scene(PlanningScene())
    backend._ensure_planner()
    registration = register_primary_scene(primary, output / 'metric_assets', sensor_session=uuid.uuid4().hex)
    robot = SapienRobotStatePort(app_root / 'assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf', resources=resources)
    robot.synchronize_primary_physics(primary)
    bodies = NativeBodyMirror(registration, robot, resources=resources)
    simulator = SapienScenePort(bodies.actors, asset_bindings=bodies.asset_bindings,
        set_attachment=bodies.set_attachment, read_attachment=bodies.read_attachment,
        apply_physics=bodies.apply_physics, read_physics=bodies.read_physics,
        make_pose=native_pose, robot_port=robot, body_port=bodies)
    planner = CuroboScenePort(backend)
    mirror = TransactionalSceneMirror(simulator, planner)
    sync = CheckpointSynchronizer(registration.world, registration.source, mirror,
        clock=time.monotonic, emit=events.emit)
    initial = sync.sync('worker_initialization_probe')
    bridge = _compile_pen_task(initial, registration.evidence, fixed, run_dir.name)
    sink = NativePrimaryExecutor(primary, emit=events.emit)
    coordinator = PickPlaceCoordinator(backend, sink)
    auditor = CuroboNativeStageAuditor(backend)

    def audit(plan, snapshot):
        started = time.monotonic()
        try:
            result = auditor(plan, snapshot)
            events.emit('swm_native_audit_passed', payload_digest=plan.payload_digest,
                snapshot_id=snapshot['snapshot_id'], passed=sorted(result.passed),
                elapsed_s=time.monotonic()-started, stages=auditor.last_evidence)
            return result
        except BaseException:
            events.emit('swm_native_audit_failed', payload_digest=plan.payload_digest,
                snapshot_id=snapshot['snapshot_id'], elapsed_s=time.monotonic()-started,
                stages=auditor.last_evidence)
            raise

    execute = SharedPrimitiveExecutor(sink, sink.feedback, clock=time.monotonic, stop=stop)
    phases = PickPlaceNativePhases(coordinator, bridge.build_task, audit, execute)
    runtime = AtomicSkillRuntime(sync, NativeAtomicBackend(phases.bindings(), execution_domain='physics'),
        clock=time.monotonic, stop=stop, goal_verifier=bridge.verify_skill)
    events.emit('swm_native_runtime_bound', task='pickplace', object_id='bi',
        input_sha256=contract['sha256'], domain='physics', skills=['grasp', 'place'],
        native_atomic_integration_verified=False, hardware_qualified=False)
    return RuntimeSession(runtime, bridge.requests(), bridge.verify_goals)


def _compile_pen_task(snapshot, evidence, scene_file, plan_id):
    import numpy as np
    from rm75_app.orchestration.multi_object_executor import SceneObjectState, TaskSceneState
    from rm75_app.pickplace.atom_task_builder import FixedSceneAtomTaskBuilder, AtomTaskBuilderConfig
    from rm75_app.llm.orchestrator import SceneObject, SceneState, _resolve_single_step
    from rm75_app.tasks.manipulation_plan import compile_resolved_steps
    from .native_tasks import CompiledNativeTask

    base = np.asarray(evidence['original_compilation']['T_world_base'])
    states, semantic_objects = {}, {}
    for oid, row in snapshot['objects'].items():
        asset = snapshot['assets'][row['asset_id']]
        metadata = {}
        infrastructure = asset.get('native_infrastructure') is True
        if infrastructure:
            metadata['swm_native_infrastructure'] = dict(
                dimensions_m=copy.deepcopy(asset['collision_dimensions_m']),
                T_object_collision=copy.deepcopy(asset['T_object_collision']))
        state = SceneObjectState(oid, asset['native_asset_name'],
            base @ np.asarray(row['measured']['T_world_object']), movable=not row['fixed'], metadata=metadata)
        states[oid] = state
        if not infrastructure:
            semantic_objects[oid] = SceneObject(oid, state.asset_name, oid, 1., False, state.pose)
    semantic = SceneState(scene_file, semantic_objects, {})
    resolved = _resolve_single_step(semantic, dict(operator='inside', source_ref='bi', target_ref='bitong'), 1)
    program = compile_resolved_steps(plan_id=plan_id, scene_file=scene_file,
        steps=[dict(source_id=resolved.source_id, source_spec='bi', operator=resolved.operator,
            target_object_id=resolved.target_object_id, target_pose=resolved.target_pose,
            primitive=resolved.primitive, place_mode=resolved.place_mode)])
    builder = FixedSceneAtomTaskBuilder(config=AtomTaskBuilderConfig(
        robot_base_world_transform=base, include_maniskill_workspace_table=False))
    return CompiledNativeTask('pickplace', program, TaskSceneState(states), builder)
