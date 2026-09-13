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

    owned = {}
    def report_cleanup():
        events.emit("swm_native_resources_released",
            primary_closed=None if "primary" not in owned else owned["primary"].closed,
            robot_mirror_closed=None if "robot" not in owned else owned["robot"].closed,
            planner_pointer_released=None if "backend" not in owned else owned["backend"]._planner is None)
    resources.callback(report_cleanup)
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
    owned["primary"] = primary
    from .native_velocity import NativeArticulationVelocity
    primary.velocity_readback = NativeArticulationVelocity(primary.env.unwrapped.agent.robot)
    backend = resources.enter_context(Curobo2Backend(Curobo2BackendConfig()))
    owned["backend"] = backend
    backend.update_scene(PlanningScene())
    backend._ensure_planner()
    registration = register_primary_scene(primary, output / 'metric_assets', sensor_session=uuid.uuid4().hex)
    robot = SapienRobotStatePort(app_root / 'assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf', resources=resources)
    owned["robot"] = robot
    robot.emit = events.emit
    events.emit(kind="swm_native_robot_physics_aligned",
        acknowledgement=robot.synchronize_primary_physics(primary))
    bodies = NativeBodyMirror(registration, robot, resources=resources)
    simulator = SapienScenePort(bodies.actors, asset_bindings=bodies.asset_bindings,
        set_attachment=bodies.set_attachment, read_attachment=bodies.read_attachment,
        apply_physics=bodies.apply_physics, read_physics=bodies.read_physics,
        make_pose=native_pose, robot_port=robot, body_port=bodies)
    planner = CuroboScenePort(backend)
    mirror = TransactionalSceneMirror(simulator, planner)
    sync = CheckpointSynchronizer(registration.world, registration.source, mirror,
        clock=time.monotonic, emit=events.emit, store=_PenCheckpointStore(output / "checkpoints"))
    initial = sync.sync('worker_initialization_probe')
    _prepare_pen_attachment(backend, planner, initial, stop, events)
    bridge = _compile_pen_task(initial, registration.evidence, fixed, run_dir.name)
    sink = NativePrimaryExecutor(primary, emit=events.emit)
    sink.object_settle_readback = registration.source.read_settle_state
    sink.closure_target = "bi"
    from .native_closure import reject_predicted_closure, screen_closure_candidate
    sink.closure_prediction = lambda: reject_predicted_closure(
        primary, registration, robot.urdf_path, target="bi", emit=events.emit,
        output=output / "closure_prediction.json")
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
    from .native_priority import CachedClosurePriority
    priority = CachedClosurePriority(backend, primary, registration, robot.urdf_path,
        directory=output / "closure_priority", emit=events.emit,
        max_predictions=backend.config.coarse_ik_batch_size)
    phases = PickPlaceNativePhases(coordinator, bridge.build_task, audit, execute,
        closure_screen=lambda candidate, snapshot, configuration: screen_closure_candidate(
            primary, registration, robot.urdf_path, candidate, snapshot, configuration,
            target="bi", emit=events.emit, directory=output / "closure_candidates"), emit=events.emit,
        candidate_priority=priority)
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


class _PenCheckpointStore:
    """Bounded immutable-revision evidence for this single-pen context."""
    def __init__(self, directory):
        self.directory = directory
        self.count = 0

    def save(self, world):
        from rm75_app.workcell.io import atomic_json
        from .scene import SceneInvalid
        if self.count >= 32:
            raise SceneInvalid('Single-pen checkpoint evidence budget exhausted')
        snapshot = world.snapshot()
        atomic_json(self.directory / f"{snapshot['revision']:04d}.json", snapshot)
        self.count += 1


def _prepare_pen_attachment(backend, planner_port, snapshot, stop, events):
    """Prepare ORIGINAL metric mesh fitting before any execution-age window.

    This temporary attachment belongs only to the private planning model. No
    executor or primary world is passed here. The same original fit/cache API
    is reused and the complete observed scene is restored even on failure.
    The subsequent runtime must still acquire and audit a fresh checkpoint.
    """
    import hashlib
    from rm75_app.planning.contracts import JointConfiguration
    from .scene import SceneInvalid

    stop.check()
    robot = snapshot['robot']
    if (snapshot.get('valid') is not True or snapshot.get('observation_domain') != 'physics'
            or robot.get('idle') is not True or robot.get('holding') != 'empty'
            or planner_port.backend is not backend
            or backend._scene.revision != snapshot['snapshot_id']
            or {obj.name for obj in backend._scene.objects} != set(snapshot['objects'])):
        raise SceneInvalid('Attachment preparation requires the complete observed idle private model')
    item = next(obj for obj in backend._scene.objects if obj.name == 'bi')
    asset = snapshot['assets'][snapshot['objects']['bi']['asset_id']]
    path = Path(asset['mesh_path']).resolve(strict=True)
    if (Path(item.metadata['visual_mesh_path']).resolve(strict=True) != path
            or list(item.metadata['visual_mesh_scale']) != [1., 1., 1.]
            or hashlib.sha256(path.read_bytes()).hexdigest() != asset['mesh_sha256']):
        raise SceneInvalid('Prepare only the same registered metric mesh and scale')
    started = time.monotonic()
    try:
        backend.attach_object('bi', JointConfiguration(tuple(robot['joint_names']), robot['positions']))
        fit = copy.deepcopy(backend._attachment_fit_report)
        if (not isinstance(fit, dict) or fit.get('object_name') != 'bi'
                or fit.get('fit_type') != 'morphit' or not fit.get('cache_file')
                or type(fit.get('fitted_spheres')) is not int or fit['fitted_spheres'] <= 0):
            raise SceneInvalid('Original registered-mesh fitting did not produce cache evidence')
    finally:
        try:
            backend.detach_object('bi')
        finally:
            restored = planner_port.apply_idle_snapshot(snapshot)
            if restored != snapshot['snapshot_id']:
                raise SceneInvalid('Attachment preparation failed complete observed-state restoration')
    stop.check()
    events.emit('swm_native_attachment_prepared', object_id='bi', source_snapshot_id=snapshot['snapshot_id'],
        mesh_sha256=asset['mesh_sha256'], elapsed_s=time.monotonic()-started, fit=fit,
        domain='private_planning_preparation', motion_executed=False, skill_verified=False)
