"""Fresh postclosure state and existing-auditor qualification of remaining lift."""
from __future__ import annotations
from dataclasses import replace
import time
import numpy as np
from .scene import SceneInvalid, digest, transform
from .native_skills import NativePrimitive, NativeStage, NativeStageState
from .skills import PlannedSkill, REQUIRED_AUDITS


class MeasuredLiftAudit:
    """Owned worker adapter, never a browser-provided success callback.

    Only the already planned lift is eligible. Replace its start with measured
    joints, retaining the endpoint and timing, then audit the entire new path
    with independently measured jaw/attachment geometry. No source setters.
    """
    def __init__(self, synchronizer, auditor, *, target, stop, emit):
        self.sync, self.auditor = synchronizer, auditor
        self.target, self.stop, self.emit = target, stop, emit

    def __call__(self, stage, trajectory):
        self.stop.check()
        if stage != 'lift':
            raise SceneInvalid('Postclosure measured audit permits remaining lift only')
        snapshot = self.sync.sync('after_close_before_lift', after=time.monotonic())
        robot = snapshot['robot']
        jaw_names = {f'gripper_{side}_{part}_Joint' for side in ('Left', 'Right')
                     for part in ('1', '2', 'Support')}
        jaw = robot.get('gripper_positions')
        names = tuple(f'joint_{i}' for i in range(1, 8))
        if (snapshot.get('valid') is not True or snapshot.get('observation_domain') != 'physics'
                or robot.get('idle') is not True or robot.get('holding') != self.target
                or tuple(robot['joint_names']) != names or not isinstance(jaw, dict)
                or set(jaw) != jaw_names or not np.isfinite(list(jaw.values())).all()
                or self.target not in snapshot['objects']):
            raise SceneInvalid('Independent complete measured holding and jaw state required for lift')
        q = np.asarray(robot['positions'], dtype=float)
        old = np.asarray(trajectory.positions, dtype=float)
        if (q.shape != (7,) or tuple(trajectory.joint_names) != names
                or old.ndim != 2 or old.shape[1] != 7 or len(old) < 2
                or not np.isfinite(q).all() or not np.isfinite(old).all()
                or np.max(np.abs(q - old[0])) > .02):
            raise SceneInvalid('Measured lift start exceeds original endpoint allowance')
        positions = old.copy()
        positions[0] = q
        path = replace(trajectory, positions=positions)
        relative = np.linalg.inv(transform(robot['T_world_tcp'])) @ transform(
            snapshot['objects'][self.target]['measured']['T_world_object'])
        held = NativeStageState(None, self.target, relative, gripper_positions=dict(jaw))
        primitive = NativePrimitive('grasp', self.target, (
            NativeStage('lift', path, state_before=held, state_after=held,
                        allow_start_contact_escape=True),))
        identity = digest(dict(stage=stage, target=self.target,
            original_positions=old.tolist(), snapshot_id=snapshot['snapshot_id']))
        plan = PlannedSkill(identity, snapshot['snapshot_id'], primitive.fingerprint(), primitive,
            snapshot['objects'][self.target]['measured']['T_world_object'],
            'measured remaining lift re-audit; not task goal verification')
        audit = self.auditor(plan, snapshot)
        if (audit.payload_digest != plan.payload_digest or audit.snapshot_id != snapshot['snapshot_id']
                or audit.passed != REQUIRED_AUDITS or primitive.fingerprint() != plan.payload_digest):
            raise SceneInvalid('Complete current measured lift audit required')
        self.stop.check()
        self.emit(kind='swm_measured_lift_audited', snapshot_id=snapshot['snapshot_id'],
            payload_digest=plan.payload_digest, original_path_digest=digest(old.tolist()),
            measured_joint_positions=q.tolist(), measured_gripper_positions=jaw,
            measured_T_tcp_object=relative.tolist(), stages=self.auditor.last_evidence,
            skill_verified=False, primary_world_mutated=False)
        return path
