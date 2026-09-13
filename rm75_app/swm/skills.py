"""Atomic, checkpointed skills shared by sorting, assembly and pushing.

Native trajectory generation and the existing shared executor are dependency
injected. A whole pick-and-place episode must NOT be registered as a grasp.
Availability is determined by installed adapters, never by merely listing names.
"""
from __future__ import annotations
from dataclasses import dataclass, field, replace
import copy
import threading
from typing import Protocol
import numpy as np
from .scene import (SceneInvalid, SyncPolicy, identifier, digest, transform,
                    pose_error, moved_between, positive)

SKILLS = ('grasp', 'place', 'push', 'pull', 'rotate')
REQUIRED_AUDITS = frozenset(('collision', 'joint_limits', 'path_continuity', 'tool_state', 'contact_policy'))


@dataclass(frozen=True)
class SkillRequest:
    skill: str
    object_id: str
    target: object = None  # Desired WORLD object pose. Grasp instead verifies holding.
    functional_pose_id: str | None = None
    position_tolerance_m: float = .006
    rotation_tolerance_rad: float = .10
    target_reference_id: str | None = None
    goal_predicate: str = "rigid_pose"

    def __post_init__(self):
        if self.skill not in SKILLS: raise ValueError('Unknown atomic skill')
        if self.goal_predicate not in ('rigid_pose', 'native_relation'):
            raise ValueError('Unknown goal predicate')
        identifier(self.object_id)
        if self.target_reference_id is not None:
            identifier(self.target_reference_id)
            if self.target_reference_id == self.object_id or self.target is None:
                raise ValueError("Relative target requires another instance and a local transform")
        if self.functional_pose_id is not None: identifier(self.functional_pose_id)
        if self.skill != 'grasp' and self.target is None: raise ValueError('Non-grasp skill requires an object goal')
        if self.target is not None:
            object.__setattr__(self, 'target', tuple(tuple(row) for row in transform(self.target).tolist()))
        positive(self.position_tolerance_m, 'position_tolerance_m')
        positive(self.rotation_tolerance_rad, 'rotation_tolerance_rad')
        if self.position_tolerance_m > .05 or self.rotation_tolerance_rad > .5:
            raise ValueError('Goal tolerances exceed the typed skill contract')

    def as_dict(self):
        result = dict(skill=self.skill, object_id=self.object_id,
                    target=None if self.target is None else [list(r) for r in self.target],
                    functional_pose_id=self.functional_pose_id,
                    position_tolerance_m=self.position_tolerance_m,
                    rotation_tolerance_rad=self.rotation_tolerance_rad)
        if self.target_reference_id is not None:
            result['target_reference_id'] = self.target_reference_id
        if self.goal_predicate != 'rigid_pose':
            result['goal_predicate'] = self.goal_predicate
        return result

    def resolve(self, snapshot):
        """Resolve a local goal against the current valid measured reference."""
        if self.target_reference_id is None:
            return self
        reference = snapshot.get('objects', {}).get(self.target_reference_id)
        if not snapshot.get('valid') or not reference or not reference.get('measured'):
            raise SceneInvalid('Target reference is not observed in a valid scene')
        measured = reference['measured']
        if measured.get('T_world_object') is None:
            raise SceneInvalid('Target reference has no measured pose')
        return replace(self, target=transform(measured['T_world_object']) @ transform(self.target),
                       target_reference_id=None)


@dataclass(frozen=True)
class SkillVerification:
    request_digest: str
    snapshot_id: str
    reached: bool
    diagnostics: dict

    def __post_init__(self):
        if type(self.reached) is not bool or not self.request_digest or not self.snapshot_id:
            raise TypeError('Verification requires explicit outcome and request/snapshot provenance')
        object.__setattr__(self, 'diagnostics', copy.deepcopy(dict(self.diagnostics)))


@dataclass(frozen=True)
class PlannedSkill:
    skill_digest: str
    source_snapshot_id: str
    payload_digest: str
    payload: object  # Trusted native adapter's path/primitive; never LLM executable text.
    expected_object_pose: object
    planner: str

    def __post_init__(self):
        transform(self.expected_object_pose)
        if not self.planner or not self.payload_digest: raise ValueError('Native plan provenance is required')


@dataclass(frozen=True)
class PlanAudit:
    payload_digest: str
    snapshot_id: str
    passed: frozenset[str]


@dataclass(frozen=True)
class ExecutionReceipt:
    command_success: bool
    started_at: float
    ended_at: float
    motion_known_complete: bool
    actual_action_id: str
    observations: object = None  # Optional measured action/object trace for physical identification.


class AtomicBackend(Protocol):
    capabilities: frozenset[str]
    execution_domain: str
    def plan(self, request: SkillRequest, snapshot: dict) -> PlannedSkill: ...
    def audit(self, plan: PlannedSkill, current_snapshot: dict) -> PlanAudit: ...
    def execute(self, plan: PlannedSkill, audit: PlanAudit) -> ExecutionReceipt: ...


@dataclass
class SkillResult:
    skill: str
    object_id: str
    status: str
    command_success: bool
    skill_verified: bool
    goal_reached: bool
    snapshot_id: str
    attempts: list = field(default_factory=list)
    verification_domain: str = 'unknown'
    reason: str = ''

    def as_dict(self): return copy.deepcopy(vars(self))


class AtomicSkillRuntime:
    """A plan/measure/audit/execute/measure loop; never a successful-command shortcut.

    Drift before execution may be retried. A post-place wrong pose requires a
    new grasp/recovery program, NOT an automatic replay of an empty-hand place.
    Unknown SDK/collision/observation errors are never caught and blindly retried.
    """
    def __init__(self, synchronizer, backend, *, clock, stop, max_replans=2,
                 max_position_tolerance_m=.006, max_rotation_tolerance_rad=.10, transition_observer=None, goal_verifier=None):
        self.sync = synchronizer; self.world = synchronizer.world; self.backend = backend
        self.clock = clock; self.stop = stop
        if type(max_replans) is not int or not 0 <= max_replans <= 8: raise ValueError('Bounded replanning required')
        self.max_replans = max_replans
        self.transition_observer = transition_observer
        if goal_verifier is not None and not callable(goal_verifier):
            raise TypeError("Trusted native goal verifier must be callable")
        self.goal_verifier = goal_verifier
        # Upper bounds on user request tolerances: callers may tighten, not loosen.
        self.max_position_tolerance = positive(max_position_tolerance_m, 'configured_position_tolerance')
        self.max_rotation_tolerance = positive(max_rotation_tolerance_rad, 'configured_rotation_tolerance')
        self._execution_lock = threading.Lock()
        self._used_actions = set()
        self.grasp_place_policy = None
        if self.world.domain != backend.execution_domain:
            raise PermissionError('Simulation/fixture adapter cannot execute a real SWM task')

    def capabilities(self):
        return {key: key in self.backend.capabilities for key in SKILLS}

    def _preconditions(self, req, snap):
        if req.skill not in self.backend.capabilities: raise NotImplementedError(f'{req.skill}: atomic adapter not installed')
        if req.goal_predicate == 'native_relation' and self.goal_verifier is None:
            raise NotImplementedError('Native relation verifier adapter not installed')
        if not snap['valid'] or req.object_id not in snap['objects']: raise SceneInvalid('Scene not synchronized')
        obj = snap['objects'][req.object_id]
        if obj['fixed']: raise ValueError('Fixed infrastructure cannot be manipulated as a free object')
        holding = snap['robot']['holding']
        if req.skill == 'grasp' and holding != 'empty': raise SceneInvalid('Grasp requires observed empty gripper')
        if req.skill == 'place' and holding != req.object_id: raise SceneInvalid('Place requires verified holding of this instance')
        if req.skill == 'push' and holding != 'empty': raise SceneInvalid('Push does not silently drop a held object')
        if req.skill in ('pull', 'rotate') and req.functional_pose_id is None:
            raise ValueError('Pull/rotate requires a registered handle/contact functional pose')
        if req.functional_pose_id is not None:
            asset = snap['assets'][obj['asset_id']]
            matches = [x for x in asset.get('functional_poses', [])
                       if x['id']==req.functional_pose_id and x['skill']==req.skill]
            if len(matches)!=1: raise ValueError('Missing/ambiguous functional pose for this object and skill')
        if req.position_tolerance_m > self.max_position_tolerance or req.rotation_tolerance_rad > self.max_rotation_tolerance:
            raise ValueError('A skill request cannot loosen configured verification tolerances')

    def run(self, request):
        if not isinstance(request, SkillRequest): raise TypeError('Expected typed SkillRequest')
        if self.grasp_place_policy is not None and request.skill in ('grasp','place'):
            raise SceneInvalid('Use the paired grasp-place sequence, not independent held-object observation')
        program_request = request
        if not self._execution_lock.acquire(blocking=False): raise RuntimeError('Another atomic skill owns this runtime')
        attempts = []; pending_plan = None; receipt = None
        try:
            for attempt in range(self.max_replans+1):
                self.stop.check()
                initial = self.sync.sync('before_'+request.skill)
                request = program_request.resolve(initial)
                self._preconditions(request, initial)
                plan = self.backend.plan(request, copy.deepcopy(initial));pending_plan = plan
                if plan.skill_digest != digest(request.as_dict()) or plan.source_snapshot_id != initial['snapshot_id']:
                    raise SceneInvalid('Native plan is not bound to the requested atomic skill and scene')
                # Slow planning never extends the capture age of the old image.
                current = self.sync.sync('pre_execute_'+request.skill)
                self._preconditions(request, current)
                row = dict(attempt=attempt, plan_snapshot=initial['snapshot_id'],
                           execution_snapshot=current['snapshot_id'], executed=False)
                attempts.append(row)
                current_request = program_request.resolve(current)
                target_changed = False
                if program_request.target_reference_id is not None:
                    p, r = pose_error(request.target, current_request.target)
                    target_changed = p > request.position_tolerance_m or r > request.rotation_tolerance_rad
                    row.update(target_reference_id=program_request.target_reference_id,
                               resolved_target=request.as_dict()['target'],
                               target_reference_error_m=p, target_reference_error_rad=r)
                if target_changed or moved_between(initial, current, self.sync.policy):
                    discard=getattr(self.backend,'discard',None)
                    if discard is not None:discard(plan)
                    pending_plan=None
                    row['reason'] = 'target_changed_during_planning' if target_changed else 'scene_changed_during_planning'
                    self.sync.emit(kind='swm_replan', **row)
                    continue
                audit = self.backend.audit(plan, copy.deepcopy(current))
                if (audit.payload_digest != plan.payload_digest or audit.snapshot_id != current['snapshot_id']
                        or not REQUIRED_AUDITS <= set(audit.passed)):
                    raise SceneInvalid('Current full path/contact/holding audit did not pass')
                if self.world.snapshot()['snapshot_id']!=current['snapshot_id']:
                    raise SceneInvalid('Scene changed during path auditing')
                if self.clock()-min(current['robot']['captured_at'],*[x['measured']['captured_at'] for x in current['objects'].values()])>self.sync.policy.max_age_s:
                    raise SceneInvalid('Observation expired during path auditing; acquire a fresh checkpoint')
                self.stop.check()
                receipt = self.backend.execute(plan, audit);pending_plan=None
                row['executed'] = True
                if (type(receipt.command_success) is not bool or receipt.motion_known_complete is not True
                        or not receipt.actual_action_id or receipt.actual_action_id in self._used_actions):
                    self.world.invalidate('unknown_motion_or_duplicate_receipt')
                    raise SceneInvalid('Execution outcome cannot be safely replayed')
                self._used_actions.add(receipt.actual_action_id)
                if not (current['robot']['captured_at'] <= receipt.started_at <= receipt.ended_at <= self.clock()):
                    self.world.invalidate('invalid_execution_clock')
                    raise SceneInvalid('Execution/capture timestamps are not comparable')
                # Even a known complete but unsuccessful command needs observation.
                post = self.sync.sync('after_'+request.skill, after=receipt.ended_at)
                if request.skill in ('push','pull','rotate') and self.transition_observer is not None:
                    if receipt.observations is not None and receipt.command_success:
                        self.transition_observer(request,receipt,initial_snapshot=current,final_snapshot=post)
                    else:
                        self.sync.emit(kind='swm_physics_fit_skipped',reason='no_measured_tool_and_object_trace')
                from .feedback import close_receipt_recording
                close_receipt_recording(receipt)
                actual = post['objects'][request.object_id]['measured']['T_world_object']
                predicted_error = pose_error(actual, plan.expected_object_pose)
                row.update(actual_action_id=receipt.actual_action_id,
                           post_snapshot=post['snapshot_id'], sim_real_error_m=predicted_error[0],
                           sim_real_error_rad=predicted_error[1], command_success=receipt.command_success,
                           resulting_swm_snapshot=self.world.snapshot()['snapshot_id'])
                if request.skill == 'grasp':
                    reached = post['robot']['holding'] == request.object_id
                    # Holding must include measured relative transform, not gripper-close status.
                    if reached:
                        relative = np.linalg.inv(transform(post['robot']['T_world_tcp'])) @ transform(actual)
                        row['measured_T_tcp_object'] = relative.tolist()
                else:
                    target_error = pose_error(actual, program_request.resolve(post).target)
                    reached = target_error[0] <= request.position_tolerance_m and target_error[1] <= request.rotation_tolerance_rad
                    row.update(goal_error_m=target_error[0], goal_error_rad=target_error[1])
                    if request.goal_predicate == 'native_relation':
                        verified_goal = self.goal_verifier(program_request, copy.deepcopy(post))
                        if (not isinstance(verified_goal, SkillVerification)
                                or verified_goal.request_digest != digest(program_request.as_dict())
                                or verified_goal.snapshot_id != post['snapshot_id']):
                            raise SceneInvalid('Native goal verification has wrong request/snapshot provenance')
                        if self.world.snapshot()['snapshot_id'] != post['snapshot_id']:
                            raise SceneInvalid('Scene changed during native goal verification')
                        reached = verified_goal.reached
                        row['native_goal_verification'] = copy.deepcopy(verified_goal.diagnostics)
                    if request.skill == 'place': reached = reached and post['robot']['holding']=='empty' 
                model_agrees = predicted_error[0] <= request.position_tolerance_m and predicted_error[1] <= request.rotation_tolerance_rad
                verified = bool(receipt.command_success and reached)
                row['model_agrees'] = model_agrees
                self.sync.emit(kind='swm_skill_verified', skill=request.skill, object_id=request.object_id,
                               goal_reached=bool(reached), skill_verified=verified, **row)
                if verified:
                    return SkillResult(request.skill, request.object_id, 'verified', True, True, True,
                                       self.world.snapshot()['snapshot_id'], attempts, self.world.domain)
                # A controller fault is never a reason to issue a second motion.
                # Wrong-placement/unknown holding requires an upper-level recovery plan.
                retry_safe = (receipt.command_success and request.skill in ('grasp','push')
                              and post['robot']['holding']=='empty')
                if not retry_safe:
                    return SkillResult(request.skill, request.object_id, 'replan_required', receipt.command_success,
                                       False, bool(reached), post['snapshot_id'], attempts, self.world.domain,
                                       'Observed outcome requires recovery; do not replay this primitive')
                # Update scene happened above regardless of success. Next attempt replans
                # from actual measurements, never from the predicted endpoint.
            return SkillResult(request.skill, request.object_id, 'replan_budget_exhausted',
                               bool(attempts and attempts[-1].get('command_success')), False, False,
                               self.world.snapshot()['snapshot_id'], attempts, self.world.domain,
                               'No downstream skill may run as though this one succeeded')
        except BaseException as exc:
            self.world.invalidate('atomic_'+type(exc).__name__)
            raise
        finally:
            try:
                from .feedback import close_receipt_recording
                close_receipt_recording(receipt)
                discard=getattr(self.backend,'discard',None)
                if pending_plan is not None and discard is not None:discard(pending_plan)
            finally:
                self._execution_lock.release()


class SWMTools:
    """Read/query/functionality/IK facade available to the high-level planner."""
    def __init__(self, world, plan_to_pose=None):
        self.world = world; self._plan_to_pose = plan_to_pose

    def get_object_pose(self, object_id):
        snap = self.world.snapshot()
        if not snap['valid']: raise SceneInvalid('Refresh scene before querying for execution')
        obj = snap['objects'][object_id]
        return dict(object_id=object_id, T_world_object=copy.deepcopy(obj['measured']['T_world_object']),
                    captured_at=obj['measured']['captured_at'], snapshot_id=snap['snapshot_id'],
                    domain=snap['observation_domain'])

    def get_functional_poses(self, object_id, skill):
        snap = self.world.snapshot()
        if not snap['valid']: raise SceneInvalid('Refresh scene before computing functionality poses')
        obj = snap['objects'][object_id]; world = transform(obj['measured']['T_world_object'])
        return [dict(id=x['id'], skill=skill, T_world_function=(world @ transform(x['T_object_function'])).tolist(),
                     object_id=object_id, snapshot_id=snap['snapshot_id'], provenance=x['provenance'])
                for x in snap['assets'][obj['asset_id']].get('functional_poses', []) if x['skill']==skill]

    def plan_to_functional_pose(self, object_id, skill, pose_id):
        matches = [x for x in self.get_functional_poses(object_id,skill) if x['id']==pose_id]
        if len(matches)!=1 or self._plan_to_pose is None:
            raise NotImplementedError('Functional pose or current-scene motion planner is unavailable')
        result = self._plan_to_pose(matches[0], self.world.snapshot())
        if self.world.snapshot()['snapshot_id'] != matches[0]['snapshot_id']:
            raise SceneInvalid('World changed while planning to a functional pose')
        return result
