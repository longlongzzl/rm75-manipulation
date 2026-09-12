"""Shared timed execution in the owned primary simulation, never real hardware."""
from __future__ import annotations
import numpy as np
from rm75_app.execution.trajectory_executor import ManiSkillTrajectoryExecutor
from .native_bootstrap import FrozenPrimaryWorld
from .scene import SceneInvalid


class NativePrimaryExecutor(ManiSkillTrajectoryExecutor):
    """Preserve the shared sink and require measured idle stage endpoints.

    Holding is deliberately not inferred here. The subsequent SWM capture uses
    the original independent contact predicate. Settling consists only of real
    primary simulator controller steps, never q/qdot or object-pose setters.
    """
    def __init__(self, primary, *, max_settle_steps=200, emit=None):
        if not isinstance(primary, FrozenPrimaryWorld) or primary.closed:
            raise TypeError('Owned live frozen primary simulation required')
        if type(max_settle_steps) is not int or not 3 <= max_settle_steps <= 200:
            raise ValueError('Bounded primary settle step budget required')
        frequency = float(primary.env.unwrapped.control_freq)
        if not np.isfinite(frequency) or frequency <= 0:
            raise ValueError('Original primary control frequency is unavailable')
        super().__init__(primary.demo, control_dt=1./frequency, stop_check=primary.stop.check)
        self.primary = primary
        self.max_settle_steps = max_settle_steps
        self.emit = emit or (lambda **row: None)
        self.last_settle_evidence = None

    def _read(self):
        self.primary.stop.check()
        if self.primary.closed:
            raise SceneInvalid('Primary simulation closed during execution')
        raw = self.primary.read_state()
        names = tuple(raw['joint_names'])
        arm = tuple(f'joint_{i}' for i in range(1, 8))
        if (raw.get('domain') != 'physics' or raw.get('source') != 'native_actor_and_joint_readback'
                or len(names) != 13 or len(set(names)) != 13 or not set(arm) <= set(names)):
            raise SceneInvalid('Primary execution feedback has wrong identity or source')
        q, velocity = np.asarray(raw['positions']), np.asarray(raw['velocities'])
        if q.shape != (13,) or velocity.shape != (13,) or not np.isfinite(q).all() or not np.isfinite(velocity).all():
            raise SceneInvalid('Primary execution feedback is incomplete or nonfinite')
        return raw, arm, q[[names.index(name) for name in arm]], velocity

    def _settle(self, stage):
        if self._last_commanded_target is None:
            raise SceneInvalid('Cannot settle without an audited trajectory endpoint')
        stable = 0
        for index in range(self.max_settle_steps):
            self.primary.stop.check()
            action = self.demo.compose_action(self._last_commanded_target, self._gripper_value)
            self.demo.step_and_render(action, tag='settle_'+stage)
            raw, names, q, velocity = self._read()
            error = float(np.max(np.abs(q-self._last_commanded_target)))
            idle = float(np.max(np.abs(velocity))) <= .001
            stable = stable + 1 if idle and error <= .02 else 0
            self.last_settle_evidence = dict(stage=stage, steps=index+1,
                primary_sequence=raw['sequence'], captured_at=raw['capture_started_at'],
                measured_positions=q.tolist(), joint_names=list(names),
                max_velocity_rad_s=float(np.max(np.abs(velocity))),
                endpoint_error_rad=error, stable_steps=stable, idle=idle)
            if stable >= 3:
                self.emit(kind='swm_primary_stage_settled', **self.last_settle_evidence)
                return
        self.emit(kind='swm_primary_settle_failed', **self.last_settle_evidence)
        raise SceneInvalid('Primary stage did not reach measured idle within settle budget')

    def execute_trajectory(self, stage, trajectory):
        self.primary.stop.check()
        super().execute_trajectory(stage, trajectory)
        self._settle(stage)

    def set_gripper(self, closed):
        self.primary.stop.check()
        if self._last_commanded_target is None:
            raise SceneInvalid('Primary gripper requires a preceding audited trajectory endpoint')
        super().set_gripper(closed)
        self._settle('gripper_close' if closed else 'gripper_open')

    def feedback(self):
        raw, names, q, velocity = self._read()
        return dict(source='measured_feedback', captured_at=raw['capture_started_at'],
            primary_sequence=raw['sequence'], joint_names=list(names), positions=q.tolist(),
            idle=bool(np.max(np.abs(velocity)) <= .001))
