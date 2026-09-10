"""Boundary-only operator controls and disturbance-safe PushT sessions.

Pause is acknowledged ONLY between complete pushes/retreats. It is not an
emergency stop. The existing STOP/controller path remains independent.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import re
import time
import numpy as np
from rm75_app.workcell.io import read_json, atomic_json, integer, finite
from .model import wrap


@dataclass(frozen=True)
class SessionPolicy:
    run_until_goal: bool = False
    max_wall_s: float = 0.  # 0: wait for goal, cancellation or a safety failure.
    position_replan_m: float = .003
    yaw_replan_rad: float = .04
    poll_s: float = .1

    @classmethod
    def from_dict(cls, raw):
        if not isinstance(raw, dict) or set(raw) - set(cls.__dataclass_fields__):
            raise ValueError('Invalid PushT session policy')
        result = cls(**raw)
        if type(result.run_until_goal) is not bool: raise ValueError('run_until_goal must be boolean')
        finite(result.max_wall_s, 'max_wall_s', 0, 86400)
        finite(result.position_replan_m, 'position_replan_m', .0001, .003)
        finite(result.yaw_replan_rad, 'yaw_replan_rad', .001, .04)
        finite(result.poll_s, 'poll_s', .02, .5)
        return result


def moved(a, b, policy):
    return (np.linalg.norm(np.asarray(a)[:2] - np.asarray(b)[:2]) > policy.position_replan_m
            or abs(wrap(a[2] - b[2])) > policy.yaw_replan_rad)


def response_is_plausible(before, push, after, *, intervention=False):
    """Conservative contamination filter, NOT a detector of every human touch."""
    if intervention: return False
    delta = np.asarray(after)[:2] - np.asarray(before)[:2]
    if not np.isfinite([*delta, before[2], after[2]]).all(): return False
    # Do not teach the response estimator an obvious external displacement.
    # Smaller ambiguous disturbances are not claimed to be detectable.
    return bool(np.linalg.norm(delta) <= max(.03, 2. * push.length_m + .01)
                and abs(wrap(after[2] - before[2])) <= max(.7, 20. * push.length_m))


def invalidate_prepared(executor):
    # Current PhysicsSession cache; no command or model/qualification mutation.
    if hasattr(executor, '_prepared_selection'): executor._prepared_selection = None
    callback = getattr(executor, 'discard_prepared_push', None)
    if callable(callback): callback()


class SessionControl:
    def __init__(self, events, policy=None):
        self.events = events
        directory = getattr(events, 'directory', None)
        self.root = Path(directory) if directory is not None else None
        if policy is None and self.root is not None and (self.root / 'session_policy.json').is_file():
            policy = read_json(self.root / 'session_policy.json', max_bytes=4096)
        self.policy = SessionPolicy.from_dict(policy or {})
        self.started = time.monotonic(); self.sequence = 0
        self.epoch = 0; self.paused = False; self.phase = 'observing'; self.step = 0
        self.last_pose = None; self._published = None

    def check_budget(self):
        if self.policy.max_wall_s and time.monotonic() - self.started > self.policy.max_wall_s:
            raise RuntimeError('interactive_session_wall_budget_exhausted')

    def state(self, phase, *, step=None, pose=None, **extra):
        self.phase = phase
        if step is not None: self.step = step
        if pose is not None: self.last_pose = list(map(float, pose))
        row = dict(schema='rm75_pusht_session_state_v1', phase=phase, step=self.step,
                   epoch=self.epoch, last_command=self.sequence, last_pose=self.last_pose,
                   run_until_goal=self.policy.run_until_goal, safe_to_adjust=(phase == 'paused'), **extra)
        if row == self._published: return row
        self._published = dict(row)
        if self.root is not None: atomic_json(self.root / 'session_status.json', row)
        self.events.emit('pusht_session_state', **row)
        return row

    def commands(self):
        if self.root is None: return []
        directory = self.root / 'session_commands'
        if not directory.exists(): return []
        paths = sorted(p for p in directory.iterdir() if re.fullmatch(r'\d{8}\.json', p.name))
        result = []
        for path in paths:
            number = int(path.stem)
            if number <= self.sequence: continue
            if number != self.sequence + len(result) + 1: raise ValueError('Session command sequence gap')
            value = read_json(path, max_bytes=4096)
            if value.get('sequence') != number: raise ValueError('Session command identity mismatch')
            if set(value) - {'sequence', 'action', 'pose'} or value.get('action') not in ('pause', 'resume', 'relocate'):
                raise ValueError('Invalid session control command')
            result.append(value)
            if len(result) >= 32: break
        return result

    def poll(self, executor, *, verification):
        changed = False
        for command in self.commands():
            action = command['action']
            if action == 'pause':
                self.paused = True; self.epoch += 1; changed = True
                invalidate_prepared(executor)
            elif action == 'resume':
                if not self.paused and self.phase != 'waiting_for_scene_change':
                    raise ValueError('Resume requires an acknowledged pause/wait')
                self.paused = False; self.epoch += 1; changed = True
                invalidate_prepared(executor)
            else:
                if not self.paused or verification != 'physics_pose':
                    raise PermissionError('T relocation is only allowed in paused, explicit physics SIM')
                from .physics_controls import relocate_idle_target
                relocate_idle_target(executor, command.get('pose'), command['sequence'])
                self.epoch += 1; changed = True
                invalidate_prepared(executor)
            self.sequence = command['sequence']
            self.state('paused' if self.paused else 'observing', command_action=action)
        return changed
