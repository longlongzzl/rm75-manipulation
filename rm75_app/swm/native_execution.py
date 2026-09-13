"""Shared timed execution in the owned primary simulation, never real hardware."""
from __future__ import annotations
import numpy as np
from rm75_app.execution.trajectory_executor import ManiSkillTrajectoryExecutor
from .native_bootstrap import FrozenPrimaryWorld, _array
from .scene import SceneInvalid


def native_endpoint_metrics(q, velocity, target):
    """Original all-joint idle and arm error metrics, shared with prediction."""
    q, velocity, target = (np.asarray(value, dtype=float) for value in (q, velocity, target))
    if (q.shape != (7,) or velocity.shape != (13,) or target.shape != (7,)
            or not all(np.isfinite(value).all() for value in (q, velocity, target))):
        raise SceneInvalid('Complete finite native endpoint feedback required')
    error = float(np.max(np.abs(q-target)))
    max_velocity = float(np.max(np.abs(velocity)))
    return error, max_velocity, max_velocity <= .001



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
        self.closure_target = None
        self.closure_prediction = None
        self.object_settle_readback = None
        self._closure_feedback = None
        self._closure_requires_reaudit = False
        self.measured_lift_audit = None
        self.paired_observation_mode = False
        self.feedback_observer = None
        self._feedback_action_id = None

    def _read(self):
        self.primary.stop.check()
        if self.primary.closed:
            raise SceneInvalid('Primary simulation closed during execution')
        raw = self.primary.read_state(include_objects=False) if self.paired_observation_mode else self.primary.read_state()
        names = tuple(raw['joint_names'])
        arm = tuple(f'joint_{i}' for i in range(1, 8))
        if (raw.get('domain') != 'physics' or raw.get('source') not in ('native_actor_and_joint_readback','native_joint_only_readback')
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
            if stage == 'gripper_close' and self._closure_feedback is not None:
                self._advance_feedback_closure()
            action = self.demo.compose_action(self._last_commanded_target, self._gripper_value)
            self.demo.step_and_render(action, tag='settle_'+stage)
            self._after_control_step('settle_'+stage)
            raw, names, q, velocity = self._read()
            error, max_velocity, idle = native_endpoint_metrics(
                q, velocity, self._last_commanded_target)
            objects = None
            ready = idle and error <= .02
            if ready and self.object_settle_readback is not None and getattr(self.primary,'object_observations_allowed',True):
                objects = self.object_settle_readback()
                if (not isinstance(objects, dict)
                        or objects.get('source') != 'native_registered_object_velocity_readback'
                        or type(objects.get('idle')) is not bool
                        or not isinstance(objects.get('objects'), dict)
                        or not objects['objects']
                        or any(not isinstance(row, dict) or type(row.get('settled')) is not bool
                               for row in objects['objects'].values())
                        or objects['idle'] != all(row['settled'] for row in objects['objects'].values())):
                    raise SceneInvalid('Complete native object settle feedback required')
                ready = objects['idle']
            if stage == 'gripper_close' and self._closure_feedback is not None:
                ready = ready and self._closure_feedback.last['in_force_band']
            stable = stable + 1 if ready else 0
            self.last_settle_evidence = dict(stage=stage, steps=index+1,
                primary_sequence=raw['sequence'], captured_at=raw['capture_started_at'],
                measured_positions=q.tolist(), joint_names=list(names),
                feedback_joint_names=list(raw["joint_names"]),
                feedback_positions_rad=list(raw["positions"]),
                feedback_velocities_rad_s=list(raw["velocities"]),
                max_velocity_rad_s=max_velocity,
                endpoint_error_rad=error, stable_steps=stable, idle=idle,
                object_settle_state=objects, velocity_evidence=raw.get("velocity_evidence")) 
            if stable >= 3:
                self.emit(kind='swm_primary_stage_settled', **self.last_settle_evidence)
                return
        self.emit(kind='swm_primary_settle_failed', measured_objects=raw.get("objects", {}),
            **self.last_settle_evidence)
        raise SceneInvalid('Primary stage did not reach measured idle within settle budget')

    def execute_trajectory(self, stage, trajectory):
        self.primary.stop.check()
        if self._closure_requires_reaudit:
            from .native_measured_lift import MeasuredLiftAudit
            from .paired_grasp_place import PredictedLiftAudit
            adapter = getattr(self, 'measured_lift_audit', None)
            required = PredictedLiftAudit if self.paired_observation_mode else MeasuredLiftAudit
            if not isinstance(adapter, required):
                raise SceneInvalid('Measured jaw and attachment lift re-audit is not installed')
            trajectory = adapter(stage, trajectory)
        super().execute_trajectory(stage, trajectory)
        self._settle(stage)
        if stage == 'lift':
            self._closure_requires_reaudit = False
        if stage == 'retreat' and self.paired_observation_mode:
            self.primary.object_observations_allowed = True
            self._settle('released_retreat')

    def set_gripper(self, closed):
        self.primary.stop.check()
        if self._last_commanded_target is None:
            raise SceneInvalid('Primary gripper requires a preceding audited trajectory endpoint')
        if closed:
            self._after_control_step('gripper_close')
            prediction = getattr(self, "closure_prediction", None)
            if prediction is not None:
                prediction()
        if closed:
            from .native_feedback_closure import NativeFeedbackClosure
            if self.paired_observation_mode:
                self.primary.object_observations_allowed = False
            self._closure_feedback = NativeFeedbackClosure()
            self._closure_requires_reaudit = True
            for _ in range(self.gripper_steps):
                self.primary.stop.check()
                self._advance_feedback_closure()
                action = self.demo.compose_action(self._last_commanded_target, self._gripper_value)
                self.demo.step_and_render(action, tag='gripper_close')
                self._after_control_step('gripper_close')
        else:
            self._closure_feedback = None
            super().set_gripper(False)
        self._settle('gripper_close' if closed else 'gripper_open')

    def _advance_feedback_closure(self):
        self.primary.stop.check()
        actor = self.primary.actors.get(self.closure_target)
        if actor is None:
            raise SceneInvalid('Bound native target required for feedback closure')
        agent = self.primary.env.unwrapped.agent
        fingers = sorted((agent.finger1_link, agent.finger2_link), key=lambda link: link.name)
        if len({link.name for link in fingers}) != 2:
            raise SceneInvalid('Distinct original finger identities required')
        vectors = [_array(agent.scene.get_pairwise_contact_forces(link, actor)).reshape(-1)
                   for link in fingers]
        self._gripper_value = self._closure_feedback.advance(vectors)
        self.emit(kind='swm_primary_feedback_closure_command',
            source='native_pairwise_contact_force_readback',
            finger_names=[link.name for link in fingers],
            **self._closure_feedback.last)

    def feedback(self):
        raw, names, q, velocity = self._read()
        if self.paired_observation_mode:
            from .native_robot import read_primary_tcp_feedback
            return dict(read_primary_tcp_feedback(self.primary),idle=bool(np.max(np.abs(velocity)) <= .001))
        return dict(source='measured_feedback', captured_at=raw['capture_started_at'],
            primary_sequence=raw['sequence'], joint_names=list(names), positions=q.tolist(),
            idle=bool(np.max(np.abs(velocity)) <= .001))

    def begin_feedback_action(self, action_id):
        if self._feedback_action_id is not None or not isinstance(action_id, str) or not action_id:
            raise SceneInvalid("Distinct actual feedback action identity required")
        self._feedback_action_id = action_id
        try:
            self._record_primary_feedback("before_atomic")
        except BaseException:
            self._feedback_action_id = None
            raise

    def end_feedback_action(self, action_id):
        if self._feedback_action_id != action_id:
            raise SceneInvalid("Feedback action release identity differs")
        self._feedback_action_id = None

    def _record_primary_feedback(self, stage):
        if self.feedback_observer is None:
            return
        if self._feedback_action_id is None:
            raise SceneInvalid("Actual action must be bound before primary feedback")
        from .native_robot import read_primary_tcp_feedback
        row = read_primary_tcp_feedback(self.primary)
        self.feedback_observer(dict(row, stage=stage, actual_action_id=self._feedback_action_id))

    def _after_control_step(self, stage):
        """Reject detected forbidden closure forces, not a sweep audit substitute.

        Pairwise forces are measured at control boundaries, not every physics
        substep. Zero resultant force is not proof of absence of collision.
        The trusted task context must bind the intended object before closing.
        """
        self._record_primary_feedback(stage)
        if stage not in ('gripper_close', 'settle_gripper_close'):
            return
        self.primary.stop.check()
        actors = dict(self.primary.actors)
        if self.closure_target not in actors:
            raise SceneInvalid('Native closure requires a bound target actor')
        agent = self.primary.env.unwrapped.agent
        links = agent.robot.links_map
        allowed = {agent.finger1_link.name, agent.finger2_link.name}
        if len(allowed) != 2 or not allowed <= set(links):
            raise SceneInvalid('Native closure requires both original finger links')
        table_id = '__swm_primary_table_contact_guard__'
        if table_id in actors:
            raise SceneInvalid('Reserved closure table identity is occupied')
        actors[table_id] = self.primary.env.unwrapped.table_scene.table
        contacts = []
        forbidden = []
        for name, link in links.items():
            for object_id, actor in actors.items():
                self.primary.stop.check()
                force = _array(agent.scene.get_pairwise_contact_forces(link, actor)).reshape(-1)
                if force.shape != (3,) or not np.isfinite(force).all():
                    raise SceneInvalid('Native closure contact feedback is incomplete or nonfinite')
                if np.any(force != 0.):
                    row = dict(robot_link=name, object_id=object_id, force_n=force.tolist())
                    contacts.append(row)
                    if object_id != self.closure_target or name not in allowed:
                        forbidden.append(row)
        self.emit(kind='swm_primary_closure_contacts', stage=stage,
            source='native_pairwise_contact_force_readback', target=self.closure_target,
            contacts=contacts, forbidden_contacts=forbidden,
            coverage='control_boundary_resultant_forces_only', skill_verified=False)
        if forbidden:
            raise SceneInvalid('Native closure detected forbidden contact')
