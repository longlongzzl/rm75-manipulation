"""Explicit simulator-only external intervention, never part of push execution."""
from __future__ import annotations
import math
import numpy as np
from .model import valid_pose
from rm75_app.workcell.transforms import vector


def relocate_idle_target(session, value, command_id):
    pose = vector(value, 3, 'intervention_pose')
    if (getattr(session, 'kind', None) not in ('tool_only_physics', 'full_arm_physics')
            or getattr(session, 'spec', {}).get('mode') != 'sim'
            or getattr(session, 'report', {}).get('hardware_connected') is not False):
        raise PermissionError('Intervention target is not the explicit offline PhysicsSession')
    if not valid_pose(pose, session.config): raise ValueError('Intervention T leaves the geometry workspace')
    base = session.base
    program = getattr(base, 'active_program', None)
    if program is not None and base.physics_time < base.program_started + program.duration - 1e-8:
        raise RuntimeError('Cannot relocate T while a push/retreat is still executing')
    before, _ = session.physical_state()
    # Check all existing tool spheres against the requested T before teleporting.
    transform = session.fk(base.read_q())
    spheres = np.asarray(session.planner_geometry['spheres'], dtype=float)
    centers = spheres[:, :3] @ transform[:3, :3].T + transform[:3, 3]
    from .model import rectangles, rotation
    r = rotation(pose[2]).T
    for x, y, width, height in rectangles(session.config):
        center_xy = pose[:2] + rotation(pose[2]) @ np.array([x, y])
        delta = centers - [*center_xy, session.motion['object_centroid_z_m']]
        local = delta.copy(); local[:, :2] = delta[:, :2] @ r.T
        half = np.array([width, height, session.motion['object_height_m']]) / 2
        distance = np.linalg.norm(np.maximum(abs(local) - half, 0), axis=1)
        if np.any(distance <= spheres[:, 3] + .002):
            raise ValueError('Requested intervention overlaps the current tool safety envelope')
    import sapien  # Lazy: no simulator, SDK or device import during API validation.
    target = sapien.Pose([float(pose[0]), float(pose[1]), session.motion['object_centroid_z_m']],
                        [math.cos(pose[2] / 2), 0, 0, math.sin(pose[2] / 2)])
    session.base.target.set_pose(target)
    # Settle after an externally imposed state; do not claim this was a push.
    session.base.target.set_linear_velocity(np.zeros(3, dtype=np.float32))
    session.base.target.set_angular_velocity(np.zeros(3, dtype=np.float32))
    row = dict(command_id=command_id, before=list(map(float, before)), after=pose.tolist(),
               source='explicit_external_sim_intervention', counted_as_push=False,
               response_fit_allowed=False, physics_time_s=base.physics_time)
    session.report['target_driven_by_physics_only']=False
    session.report['target_teleported_only_for_explicit_intervention']=True
    session.report['all_push_dynamics_physics_driven']=True
    session.report.setdefault('external_interventions', []).append(row)
    session.events.emit('pusht_external_intervention', **row)
