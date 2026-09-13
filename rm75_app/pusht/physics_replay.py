"""Validated saved joint-path replay inputs for an isolated tool/T physics test.

No hardware, solver, target teleporting, or success from a surrogate prediction.
The full tool is kinematically driven; arm servo dynamics are NOT simulated.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from rm75_app.planning.gripper_collision import _rpy_matrix, _axis_angle_matrix


STAGES = ('approach', 'descend', 'contact', 'push', 'retreat')


def forbidden_target_contact(link, stage, allowed):
    """Physical target contact has the same link scope as the GPU audit."""
    return link not in allowed or stage not in ('contact','push','retreat','post_settle')


class TcpFK:
    """URDF base_link -> gripper_tcp rigid chain, with the original seven joints."""
    def __init__(self, urdf):
        root = ET.parse(Path(urdf)).getroot()
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain = []; child = 'gripper_tcp'
        while child != 'base_link':
            joint = parents[child]; chain.append(joint); child = joint.find('parent').get('link')
        self.chain = []
        for joint in reversed(chain):
            origin = joint.find('origin'); transform = np.eye(4)
            if origin is not None:
                transform[:3, :3] = _rpy_matrix(np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
                transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            index = None; axis = None
            if joint.get('type') != 'fixed':
                name = joint.get('name')
                if name not in [f'joint_{i}' for i in range(1, 8)]:
                    raise ValueError('Unexpected active joint on TCP chain: '+str(name))
                index = int(name.split('_')[-1])-1
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            self.chain.append((transform, index, axis))
        if sorted(i for _, i, _ in self.chain if i is not None) != list(range(7)):
            raise ValueError('TCP chain must contain all seven original arm joints')

    def __call__(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('Expected seven finite joint positions')
        result = np.eye(4)
        for origin, index, axis in self.chain:
            result = result @ origin
            if index is not None:
                rotation = np.eye(4); rotation[:3, :3] = _axis_angle_matrix(axis.copy(), q[index])
                result = result @ rotation
        return result


class TimedProgram:
    def __init__(self, result, fk):
        if (result.get('complete_chain') is not True or result.get('validation_success') is not True
                or any(result.get(k) is not False for k in
                       ('execute_real', 'hardware_connected', 'hardware_profile_qualified'))):
            raise ValueError('Need a completed no-hardware GPU plan')
        if tuple(row['stage'] for row in result['stages']) != STAGES:
            raise ValueError('All five stages must be preserved in their original order')
        self.fk = fk; self.rows = []; self.duration = 0.; previous = None
        for row in result['stages']:
            q = np.asarray(row['positions'], dtype=float); t = np.asarray(row['times'], dtype=float)
            if (q.ndim != 2 or q.shape[1] != 7 or len(q) < 2 or t.shape != (len(q),)
                    or not np.isfinite(q).all() or not np.isfinite(t).all()
                    or abs(t[0]) > 1e-9 or np.any(np.diff(t) <= 0)):
                raise ValueError('Invalid original timed joint path')
            if previous is not None and np.max(abs(previous-q[0])) > 1e-5:
                raise ValueError('Cannot teleport across a stage discontinuity')
            self.rows.append((row['stage'], self.duration, q.copy(), t.copy()))
            self.duration += t[-1]; previous = q[-1]
        self.initial = self.rows[0][2][0].copy()

    def configuration(self, time_s):
        if not np.isfinite(time_s): raise ValueError('Finite replay time required')
        for name, offset, q, times in self.rows:
            if time_s <= offset+times[-1]: break
        t = np.clip(time_s-offset, 0, times[-1])
        point = np.array([np.interp(t, times, q[:, j]) for j in range(7)])
        return name, point

    def sample(self, time_s):
        name,point=self.configuration(time_s)
        return name,self.fk(point)

    def stage_programs(self):
        """Original timed stages, each clamped so time cannot enter its successor."""
        return tuple(TimedStageProgram(name,q,t,self.fk) for name,_,q,t in self.rows)


class TimedStageProgram:
    """One already-validated stage; no replanning or new execution permission."""
    def __init__(self,name,q,t,fk):
        q=np.asarray(q,dtype=float);t=np.asarray(t,dtype=float)
        if (name not in STAGES or q.ndim!=2 or q.shape[1]!=7 or not 2<=len(q)<=100000
                or t.shape!=(len(q),) or not np.isfinite(q).all() or not np.isfinite(t).all()
                or abs(t[0])>1e-9 or np.any(np.diff(t)<=0)):
            raise ValueError('Invalid timed stage program')
        self.hold_stage=name;self.positions=q.copy();self.times=t.copy();self.fk=fk
        self.duration=float(t[-1]);self.initial=q[0].copy()

    def configuration(self,time_s):
        if not np.isfinite(time_s):raise ValueError('Finite replay time required')
        t=np.clip(time_s,0,self.duration)
        return self.hold_stage,np.array([
            np.interp(t,self.times,self.positions[:,j]) for j in range(7)])

    def sample(self,time_s):
        name,q=self.configuration(time_s)
        return name,self.fk(q)


def audit_endpoints(program, points, rotation, position_tolerance, orientation_tolerance):
    """Check replay FK against the original five stage targets and tolerances."""
    if tuple(row[0] for row in points) != STAGES:
        raise ValueError('Endpoint audit requires all five original targets')
    output = []
    for (name, _, q, _), (_, xyz, *_) in zip(program.rows, points):
        transform = program.fk(q[-1])
        position_error = float(np.linalg.norm(transform[:3, 3]-xyz))
        orientation_error = float(np.arccos(np.clip(
            (np.trace(transform[:3, :3].T@rotation)-1)/2, -1, 1)))
        if (not np.isfinite([position_error, orientation_error]).all()
                or position_error > position_tolerance or orientation_error > orientation_tolerance):
            raise ValueError('Replay FK differs from original target: '+name)
        output.append(dict(stage=name, position_error_m=position_error,
                           orientation_error_rad=orientation_error))
    return output


def measured_outcome(initial, final, push, goal, config):
    """Measured displacement and ORIGINAL task tolerances; no promotion of a short push."""
    from .model import error, reached, wrap, valid_pose
    a, b = np.asarray(initial), np.asarray(final); delta = b[:2]-a[:2]
    direction = np.asarray(push.direction)
    return dict(displacement_m=float(np.linalg.norm(delta)),
        forward_displacement_m=float(delta@direction),
        lateral_displacement_m=float(delta@np.array([-direction[1], direction[0]])),
        yaw_change_rad=wrap(b[2]-a[2]), initial_goal_error_m=error(a, goal, config),
        final_goal_error_m=error(b, goal, config),
        final_goal_position_error_m=float(np.linalg.norm(b[:2]-np.asarray(goal)[:2])),
        final_goal_yaw_error_rad=abs(wrap(b[2]-goal[2])),
        final_pose_valid=bool(valid_pose(b, config)),
        goal_reached=bool(reached(b, goal, config)))
