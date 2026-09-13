"""Bounded force-feedback jaw commands shared by owned simulated worlds.

Force bands regulate the command only. They never certify attachment, grasp
angles, collision freedom, or permission to execute a subsequent trajectory.
"""
from __future__ import annotations
import numpy as np
from .scene import SceneInvalid


class NativeFeedbackClosure:
    policy_id = 'native_feedback_closure_1'

    def __init__(self):
        self.command = -1.
        self.contact_mode = False
        self.ticks = 0
        self.last = None

    def advance(self, finger_forces):
        forces = np.asarray(finger_forces, dtype=float)
        if forces.shape != (2, 3) or not np.isfinite(forces).all():
            raise SceneInvalid('Two finite measured finger force vectors required')
        if self.ticks >= 220:
            raise SceneInvalid('Original close plus hold feedback budget exhausted')
        magnitudes = np.linalg.norm(forces, axis=1)
        if not np.isfinite(magnitudes).all():
            raise SceneInvalid('Finite measured finger force magnitudes required')
        self.ticks += 1
        low, high = float(min(magnitudes)), float(max(magnitudes))
        self.contact_mode = self.contact_mode or high > .1
        in_band = low >= .5 and high <= 2.
        if not self.contact_mode:
            self.command = min(1., self.command + .04)
        elif not in_band:
            error = 1. - (high if high > 4. else low)
            angle = (self.command + 1.) * .91 / 2.
            angle += float(np.clip(error * .0005, -.0005, .0005))
            self.command = float(np.clip(2. * angle / .91 - 1., -1., 1.))
        self.last = dict(policy_id=self.policy_id, control_tick=self.ticks,
            semantic_command=self.command, pre_step_forces_n=forces.tolist(),
            pre_step_force_magnitudes_n=magnitudes.tolist(), in_force_band=in_band,
            holding_qualified=False)
        return self.command
