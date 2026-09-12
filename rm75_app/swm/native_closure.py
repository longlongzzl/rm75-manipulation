"""Owned CPU closure prediction: a rejection gate, never a success proof."""
from __future__ import annotations

import copy
import json
import time
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path

import numpy as np

from .scene import SceneInvalid, SceneWorldModel, SyncPolicy


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


def reject_predicted_closure(primary, registration, urdf_path, *, target, emit, output):
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
    robot = None
    try:
        if target not in registration.source.bindings:
            raise SceneInvalid('Closure prediction requires registered target')
        manifest = Path(registration.world.assets[target]['mesh_path']).parent / 'manifest.json'
        if file_digest(manifest) != registration.evidence['manifest_sha256']:
            raise SceneInvalid('Original registered manifest changed')
        world = SceneWorldModel(json.loads(manifest.read_text()))
        world.check_assets()
        after = time.monotonic()
        batch = registration.source.capture(tuple(registration.source.bindings),
            after=after, boundary='native_preclose_prediction')
        prepared = world.prepare_checkpoint(batch, after=after, now=time.monotonic(),
            policy=SyncPolicy(), boundary='native_preclose_prediction')
        snapshot = world.commit_checkpoint(prepared)
        report['snapshot'] = snapshot
        before = primary.read_state()
        drives = read_primary_drive_state(primary)
        agent = primary.env.unwrapped.agent
        fingers = {agent.finger1_link.name, agent.finger2_link.name}
        if len(fingers) != 2 or not fingers <= set(agent.robot.links_map):
            raise SceneInvalid('Closure prediction requires original finger identity')
        controller = agent.controller.controllers['gripper']
        cfg = controller.config
        if cfg.use_delta or cfg.use_target or cfg.interpolate:
            raise SceneInvalid('Unsupported closure controller state')
        import torch
        command = controller._preprocess_action(
            torch.ones((1, controller.effective_dof), device=controller.device))
        targets = torch.zeros_like(controller._target_qpos)
        targets[:, controller.control_joint_indices] = command
        targets[:, controller.mimic_joint_indices] = (
            targets[:, controller.mimic_control_joint_indices]
            * controller._multiplier[None, :] + controller._offset[None, :])
        values = _array(targets).reshape(-1)
        if len(values) != len(controller.joints):
            raise SceneInvalid('Closure drive target identity is incomplete')
        closed = copy.deepcopy(drives)
        for joint, value in zip(controller.joints, values):
            closed['position_targets'][closed['joint_names'].index(joint.name)] = float(value)
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
            entities = [link.entity for link in robot._robot.get_links()]
            from mani_skill.utils.sapien_utils import compute_total_impulse
            dt = closed['physics_timestep_s']
            substeps = round(closed['simulation_frequency_hz'] / closed['control_frequency_hz'])
            for tick in range(20):
                apply_native_drive_state(robot._robot, robot._physics_system, closed)
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
                    report['steps'].append(dict(control_tick=tick+1, substep=substep+1,
                        q=_array(robot._robot.get_qpos()).reshape(-1).tolist(),
                        qdot=_array(robot._robot.get_qvel()).reshape(-1).tolist(), contacts=contacts))
                    if forbidden:
                        report['forbidden_contacts'] = forbidden
                        report['phase'] = 'rejected'
                        raise SceneInvalid('Native closure prediction detected forbidden contact')
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
                unchanged = unchanged and read_primary_drive_state(primary) == drives
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
