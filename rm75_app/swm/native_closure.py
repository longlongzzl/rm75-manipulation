"""Owned CPU closure prediction: a rejection gate, never a success proof."""
from __future__ import annotations

import copy
import json
import time
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path

import numpy as np

from .scene import SceneInvalid, SceneWorldModel, SyncPolicy, digest


class NativeClosureRejected(SceneInvalid):
    """An explicit negative prediction, distinct from unavailable model state."""


def forbidden_object_contact(names, force, separations, *, objects, fingers, target):
    """Classify robot/object contacts only; self-collision remains auditor-owned.

    Retain zero-impulse penetration. Positive-distance proximity alone is not
    impact. There is deliberately no epsilon for discarding measured forces.
    """
    force = np.asarray(force, dtype=float)
    distances = np.asarray(separations, dtype=float)
    if (len(names) != 2 or force.shape != (3,) or not np.isfinite(force).all()
            or distances.ndim != 1 or not len(distances)
            or not np.isfinite(distances).all()):
        raise SceneInvalid('Incomplete native closure contact evidence')
    object_ids = set(names) & set(objects)
    if not object_ids:
        return False
    if len(object_ids) != 1:
        raise SceneInvalid('Ambiguous robot/object contact identity')
    object_id = next(iter(object_ids))
    link = names[1] if names[0] == object_id else names[0]
    allowed = object_id == target and link in fingers
    return not allowed and bool(np.any(force != 0.) or np.any(distances < 0.))


def reject_predicted_closure(primary, registration, urdf_path, *, target, emit, output,
                             candidate_snapshot=None, candidate_configuration=None):
    """Capture fresh idle state and step only a separate, fully populated scene.

    This bounded negative screen does not qualify contact-model agreement,
    settling, attachment or skill success. All original execution guards remain.
    """
    from .native_bootstrap import _array
    from .native_robot import read_primary_drive_state
    from .native_robot_mirror import SapienRobotStatePort, apply_native_drive_state
    from .native_body_mirror import NativeBodyMirror, native_pose
    from .native_scene import SapienScenePort
    from .native_registration import file_digest

    report = dict(domain='private_CPU_PhysX_closure_rejection',
                  model_qualified=False, skill_qualified=False,
                  hardware_connected=False, phase='capture', steps=[])
    before = None
    drives = None
    robot = None
    try:
        if target not in registration.source.bindings:
            raise SceneInvalid('Closure prediction requires registered target')
        manifest = Path(registration.world.assets[target]['mesh_path']).parent / 'manifest.json'
        if file_digest(manifest) != registration.evidence['manifest_sha256']:
            raise SceneInvalid('Original registered manifest changed')
        world = SceneWorldModel(json.loads(manifest.read_text()))
        world.check_assets()
        if (candidate_snapshot is None) != (candidate_configuration is None):
            raise SceneInvalid('Candidate snapshot and endpoint must be supplied together')
        if candidate_snapshot is None:
            after = time.monotonic()
            batch = registration.source.capture(tuple(registration.source.bindings),
                after=after, boundary='native_preclose_prediction')
            prepared = world.prepare_checkpoint(batch, after=after, now=time.monotonic(),
                policy=SyncPolicy(), boundary='native_preclose_prediction')
            snapshot = world.commit_checkpoint(prepared)
        else:
            snapshot = copy.deepcopy(candidate_snapshot)
            if (snapshot.get('valid') is not True or snapshot.get('observation_domain') != 'physics'
                    or snapshot['robot'].get('idle') is not True
                    or snapshot['robot'].get('holding') != 'empty'
                    or set(snapshot['objects']) != set(registration.source.bindings)):
                raise SceneInvalid('Candidate prediction requires complete observed idle physics scene')
            report['domain'] = 'private_CPU_PhysX_candidate_hypothesis'
        report['snapshot'] = snapshot
        before = primary.read_state()
        drives = read_primary_drive_state(primary)
        agent = primary.env.unwrapped.agent
        fingers = {agent.finger1_link.name, agent.finger2_link.name}
        if len(fingers) != 2 or not fingers <= set(agent.robot.links_map):
            raise SceneInvalid('Closure prediction requires original finger identity')
        controller = agent.controller.controllers['gripper']
        closed = original_gripper_drive_targets(controller, drives, 1.)
        report['closed_drive_targets'] = closed
        report['phase'] = 'private_prediction'
        with ExitStack() as resources:
            robot = SapienRobotStatePort(urdf_path, resources=resources)
            robot.synchronize_primary_physics(primary)
            bodies = NativeBodyMirror(replace(registration, world=world), robot, resources=resources)
            port = SapienScenePort(bodies.actors, asset_bindings=bodies.asset_bindings,
                set_attachment=bodies.set_attachment, read_attachment=bodies.read_attachment,
                apply_physics=bodies.apply_physics, read_physics=bodies.read_physics,
                make_pose=native_pose, robot_port=robot, body_port=bodies)
            port.apply_idle_snapshot(snapshot)
            report['initial_robot_ack'] = robot.acknowledgement
            report['initial_body_ack'] = bodies.readback()
            report['object_ids'] = sorted(bodies.actors)
            if candidate_configuration is not None:
                report['hypothesis_initialization'] = apply_candidate_endpoint(
                    robot._robot, candidate_configuration, closed)
            entities = [link.entity for link in robot._robot.get_links()]
            from mani_skill.utils.sapien_utils import compute_total_impulse
            dt = closed['physics_timestep_s']
            substeps = round(closed['simulation_frequency_hz'] / closed['control_frequency_hz'])
            def advance_control(tick, command, stage):
                apply_native_drive_state(robot._robot, robot._physics_system, command)
                for substep in range(substeps):
                    primary.stop.check()
                    robot._scene.step()
                    contacts = []
                    forbidden = []
                    for contact in robot._physics_system.get_contacts():
                        if not any(body.entity in entities for body in contact.bodies):
                            continue
                        names = [body.entity.name for body in contact.bodies]
                        force = (compute_total_impulse([(contact, True)]) / dt).tolist()
                        separations = [getattr(point, 'separation', None) for point in contact.points]
                        row = dict(body_names=names, force_in_base_n=force, separations_m=separations)
                        contacts.append(row)
                        if forbidden_object_contact(names, force, separations,
                                objects=bodies.actors, fingers=fingers, target=target):
                            forbidden.append(row)
                    report['steps'].append(dict(stage=stage, control_tick=tick+1, substep=substep+1,
                        q=_array(robot._robot.get_qpos()).reshape(-1).tolist(),
                        qdot=_array(robot._robot.get_qvel()).reshape(-1).tolist(), contacts=contacts))
                    if forbidden:
                        report['forbidden_contacts'] = forbidden
                        report['phase'] = 'rejected'
                        report['rejection_stage'] = stage
                        raise NativeClosureRejected('Native closure prediction detected forbidden contact')
                return report['steps'][-1]
            if candidate_configuration is not None:
                report['phase'] = 'candidate_open_preparation'
                opened = original_gripper_drive_targets(controller, closed, -1.)
                report['candidate_open_drive_targets'] = opened
                names = [joint.name for joint in robot._robot.get_active_joints()]
                try:
                    report['candidate_open_preparation'] = settle_candidate_open(
                        lambda tick: advance_control(tick, opened, 'candidate_open_hold'),
                        names, candidate_configuration.positions)
                except NativeClosureRejected as error:
                    report['candidate_open_preparation'] = getattr(error, 'evidence', {'completed': False})
                    report['phase'] = 'rejected'
                    report['rejection_stage'] = 'candidate_open_hold'
                    raise
            report['phase'] = 'private_closure_steps'
            for tick in range(20):
                advance_control(tick, closed, 'gripper_close')
            report['phase'] = 'no_forbidden_contact_predicted_not_qualified'
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        try:
            if before is not None:
                final = primary.read_state()
                unchanged = all(final[key] == before[key]
                    for key in ('positions', 'velocities', 'objects'))
                unchanged = unchanged and (drives is None or read_primary_drive_state(primary) == drives)
                report['primary_unchanged_while_private_stepped'] = unchanged
                if not unchanged:
                    raise SceneInvalid('Private prediction changed primary state')
        finally:
            report['private_robot_closed'] = None if robot is None else robot.closed
            Path(output).write_text(json.dumps(report, indent=2))
            emit(kind='swm_native_closure_prediction', phase=report['phase'],
                 evidence_path=str(output), model_qualified=False, skill_qualified=False,
                 private_robot_closed=report['private_robot_closed'],
                 primary_unchanged=report.get('primary_unchanged_while_private_stepped'))


def apply_candidate_endpoint(robot, configuration, closed):
    """Private hypothetical initialization, NOT measured feedback or settling.

    Place the arm at a proposed endpoint and assume rest for this negative
    screen only. Actual approach/grasp, settling and fresh preclose prediction
    remain mandatory. No object pose or measured SWM snapshot is changed.
    """
    from .native_robot_mirror import validate_native_drive_state
    arm = tuple(f'joint_{i}' for i in range(1, 8))
    jaw = {f'gripper_{side}_{part}_Joint' for side in ('Left', 'Right')
           for part in ('1', '2', 'Support')}
    joints = list(robot.get_active_joints())
    names = [joint.name for joint in joints]
    positions = np.asarray(configuration.positions, dtype=float)
    validate_native_drive_state(closed)
    if (tuple(configuration.names) != arm or positions.shape != (7,)
            or not np.isfinite(positions).all() or len(names) != 13
            or len(set(names)) != 13 or set(names) != set(arm) | jaw):
        raise SceneInvalid('Candidate endpoint requires original complete joint identity')
    q = np.asarray(robot.get_qpos(), dtype=float).reshape(-1).copy()
    if q.shape != (13,) or not np.isfinite(q).all():
        raise SceneInvalid('Private candidate initialization lacks native joint state')
    for name, position in zip(arm, positions):
        limits = np.asarray(joints[names.index(name)].limits, dtype=float)
        if (limits.shape != (1, 2) or not np.isfinite(limits).all()
                or not limits[0, 0] <= position <= limits[0, 1]):
            raise SceneInvalid('Candidate endpoint violates native joint limits')
        q[names.index(name)] = position
    velocity = np.zeros(13, dtype=np.float32)
    robot.set_qpos(q.astype(np.float32))
    robot.set_qvel(velocity)
    actual = np.asarray(robot.get_qpos(), dtype=float).reshape(-1)
    actual_velocity = np.asarray(robot.get_qvel(), dtype=float).reshape(-1)
    if (actual.shape != (13,) or actual_velocity.shape != (13,)
            or not np.isfinite(actual).all() or not np.isfinite(actual_velocity).all()
            or np.max(np.abs(actual-q)) > 1e-6 or np.any(actual_velocity != 0.)):
        raise SceneInvalid('Private candidate initialization did not apply')
    for name, position in zip(arm, positions):
        closed['position_targets'][closed['joint_names'].index(name)] = float(position)
    return dict(source='hypothetical_candidate_endpoint_not_observed',
        assumption='stationary_endpoint_not_executed_approach',
        joint_names=names, positions=actual.tolist(), velocities=actual_velocity.tolist(),
        measured=False, primary_world_mutated=False)


def screen_closure_candidate(primary, registration, urdf_path, candidate, snapshot,
                             configuration, *, target, emit, directory):
    """Reject this candidate only for explicit contact or bounded idle failure.

    Identity failures, missing model state, cancellation and ownership errors
    propagate; none may be interpreted as another ordinary infeasible grasp.
    """
    identity = digest(dict(candidate_id=candidate.candidate_id,
        snapshot_id=snapshot['snapshot_id'], joint_names=list(configuration.names),
        positions=np.asarray(configuration.positions).tolist()))
    path = Path(directory) / (identity + '.json')
    path.parent.mkdir(exist_ok=True)
    try:
        reject_predicted_closure(primary, registration, urdf_path, target=target,
            emit=emit, output=path, candidate_snapshot=snapshot,
            candidate_configuration=configuration)
    except NativeClosureRejected:
        emit(kind='swm_native_closure_candidate_rejected', candidate_id=candidate.candidate_id,
            snapshot_id=snapshot['snapshot_id'], evidence_path=str(path), model_qualified=False)
        return False
    emit(kind='swm_native_closure_candidate_not_rejected', candidate_id=candidate.candidate_id,
        snapshot_id=snapshot['snapshot_id'], evidence_path=str(path), model_qualified=False)
    return True


def original_gripper_drive_targets(controller, baseline, value):
    """Pure original absolute mimic command transform, with no source setters."""
    from .native_bootstrap import _array
    from .native_robot_mirror import validate_native_drive_state
    validate_native_drive_state(baseline)
    cfg = controller.config
    if value not in (-1., 1.) or cfg.use_delta or cfg.use_target or cfg.interpolate:
        raise SceneInvalid('Unsupported closure controller state')
    import torch
    command = controller._preprocess_action(
        torch.ones((1, controller.effective_dof), device=controller.device) * value)
    targets = torch.zeros_like(controller._target_qpos)
    targets[:, controller.control_joint_indices] = command
    targets[:, controller.mimic_joint_indices] = (
        targets[:, controller.mimic_control_joint_indices]
        * controller._multiplier[None, :] + controller._offset[None, :])
    values = _array(targets).reshape(-1)
    names = [joint.name for joint in controller.joints]
    jaw = {f'gripper_{side}_{part}_Joint' for side in ('Left', 'Right')
           for part in ('1', '2', 'Support')}
    if len(values) != len(names) or not names or len(set(names)) != len(names) or not set(names) <= jaw:
        raise SceneInvalid('Closure drive target identity is incomplete')
    result = copy.deepcopy(baseline)
    for name, position in zip(names, values):
        result['position_targets'][result['joint_names'].index(name)] = float(position)
    validate_native_drive_state(result)
    return result


def settle_candidate_open(step, names, arm_target, *, max_steps=200):
    """Require actual private open-hold feedback under original idle criteria.

    This prepares a hypothetical endpoint, not an executed primary approach.
    No q/qdot setters are used here. The callback steps only its owned scene
    and must enforce the same forbidden-contact policy on every substep.
    """
    from .native_execution import native_endpoint_metrics
    arm = tuple(f'joint_{i}' for i in range(1, 8))
    jaw = {f'gripper_{side}_{part}_Joint' for side in ('Left', 'Right')
           for part in ('1', '2', 'Support')}
    if (type(max_steps) is not int or not 3 <= max_steps <= 200
            or len(names) != 13 or len(set(names)) != 13 or set(names) != set(arm) | jaw):
        raise SceneInvalid('Original private open-hold identity and step budget required')
    stable = 0
    for tick in range(max_steps):
        row = step(tick)
        q = np.asarray(row['q'], dtype=float)
        if q.shape != (13,) or not np.isfinite(q).all():
            raise SceneInvalid('Private open-hold feedback is incomplete')
        velocity = np.asarray(row['qdot'], dtype=float)
        error, max_velocity, idle = native_endpoint_metrics(
            q[[names.index(name) for name in arm]], velocity, arm_target)
        stable = stable + 1 if idle and error <= .02 else 0
        evidence = dict(source='native_private_open_hold_readback', completed=stable >= 3,
            control_steps=tick+1, stable_steps=stable, endpoint_error_rad=error,
            max_velocity_rad_s=max_velocity, idle=idle, joint_names=list(names),
            positions=q.tolist(), velocities=velocity.tolist(), primary_world_mutated=False)
        if stable >= 3:
            return evidence
    error = NativeClosureRejected('Candidate open hold did not reach original idle criterion')
    error.evidence = evidence
    raise error
