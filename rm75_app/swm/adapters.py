"""Explicit integration ports for EXISTING solvers and the shared executor.

No new robot driver is defined. Atomic callbacks must be extracted at real native
skill boundaries, not wrap a full pick/place episode and relabel it 'grasp'.
Callbacks cannot be supplied by the browser or generated LLM code.
"""
from __future__ import annotations
from dataclasses import dataclass
import copy
from .scene import SceneInvalid, transform, digest
from .skills import SKILLS, PlannedSkill, PlanAudit, ExecutionReceipt, REQUIRED_AUDITS


@dataclass(frozen=True)
class NativeAtomicBinding:
    plan: object
    audit: object
    execute: object
    native_boundary: str

    def __post_init__(self):
        if not all(callable(x) for x in (self.plan,self.audit,self.execute)) or not self.native_boundary:
            raise ValueError('A native atomic adapter needs actual plan/audit/execute bindings')


class NativeAtomicBackend:
    """Bind the same native planning/execution implementation to all tasks."""
    def __init__(self, bindings, *, execution_domain):
        if set(bindings)-set(SKILLS):raise ValueError('Unknown atomic bindings')
        if execution_domain not in ('real','physics','fixture'):raise ValueError('Explicit adapter domain required')
        if not all(isinstance(x,NativeAtomicBinding) for x in bindings.values()):raise TypeError('Typed binding required')
        self.bindings=dict(bindings);self.capabilities=frozenset(bindings)
        self.execution_domain=execution_domain;self._owners={}

    def plan(self,request,snapshot):
        binding=self.bindings[request.skill]
        result=binding.plan(request,copy.deepcopy(snapshot))
        if not isinstance(result,PlannedSkill):raise TypeError('Native solver must return a bound atomic plan')
        if result.source_snapshot_id!=snapshot['snapshot_id'] or result.skill_digest!=digest(request.as_dict()):
            raise SceneInvalid('Native solver returned stale/other skill plan')
        self._owners[result.payload_digest]=binding
        return result

    def discard(self,plan):
        self._owners.pop(plan.payload_digest,None)

    def audit(self,plan,current_snapshot):
        binding=self._owners[plan.payload_digest]
        result=binding.audit(plan,copy.deepcopy(current_snapshot))
        if not isinstance(result,PlanAudit):raise TypeError('Explicit current-scene trajectory audit required')
        return result

    def execute(self,plan,audit):
        # Runtime supplied current-scene audit; executor still enforces its own
        # connection/arming/start-state/timing/stop safeguards independently.
        if audit.payload_digest!=plan.payload_digest or not REQUIRED_AUDITS<=set(audit.passed):
            raise SceneInvalid('Cannot dispatch an unaudited native primitive')
        binding=self._owners.pop(plan.payload_digest)
        result=binding.execute(plan,audit)
        if not isinstance(result,ExecutionReceipt):raise TypeError('Actual execution feedback receipt required')
        return result


class TransactionalSceneMirror:
    """Synchronize simulator and planning collision state at IDLE checkpoints.

    Both ports apply/validate the entire snapshot including held-object attachment
    derived from the measured TCP/object relation. A rejected port latches this
    mirror invalid; no acknowledgement is returned, and SWM disallows execution.
    Reinitializing both ports with a complete checkpoint is the only recovery.
    """
    def __init__(self, simulator_port, planner_port):
        self.simulator=simulator_port;self.planner=planner_port;self.valid=False
    def replace_scene(self,snapshot):
        self.valid=False
        if snapshot['robot']['idle'] is not True:raise SceneInvalid('No synchronization while moving')
        for port in (self.simulator,self.planner):
            returned=port.apply_idle_snapshot(copy.deepcopy(snapshot))
            if returned!=snapshot['snapshot_id']:
                raise SceneInvalid('Scene mirror did not apply exact pose/attachment/physics revision')
        self.valid=True
        return snapshot['snapshot_id']


def to_task_scene_state(snapshot):
    """Reuse existing multi-object scene types; do not create alternate identities."""
    from rm75_app.orchestration.multi_object_executor import TaskSceneState, SceneObjectState, ObjectLifecycle
    if not snapshot['valid']:raise SceneInvalid('SWM not synchronized')
    objects={}
    for oid,obj in snapshot['objects'].items():
        objects[oid]=SceneObjectState(oid,obj['asset_id'],obj['measured']['T_world_object'],
            lifecycle=ObjectLifecycle.HELD if obj['lifecycle']=='held' else ObjectLifecycle.AVAILABLE,
            movable=not obj['fixed'],metadata=dict(swm_observation=obj['measured'],
                mesh_sha256=snapshot['assets'][obj['asset_id']]['mesh_sha256']))
    robot=snapshot['robot']
    return TaskSceneState(objects,revision=snapshot['revision'],backend_revision=snapshot['snapshot_id'],
                          joint_names=tuple(robot['joint_names']),joint_positions=robot['positions'],
                          metadata=dict(swm_domain=snapshot['observation_domain'],physics_revision=snapshot['physics_revision']))


def compose_task(task, object_targets, *, source_to_instance=None):
    """Compile task semantics to atom names, NEVER run native code here.

    object_targets must come from the existing PickPlace placement rules or Jimu
    compiled WORLD targets/topological ordering. Builder Y-up coordinates must
    already be transformed by the original task/compiler calibration.
    """
    from .skills import SkillRequest
    if task not in ('pickplace','magnetic','pusht'):raise ValueError('Unknown scenario')
    if not object_targets:raise ValueError('No target bindings')
    source_to_instance=source_to_instance or {}
    steps=[]
    for source,target in object_targets:
        oid=source_to_instance.get(source,source);transform(target)
        if task=='pusht':steps.append(SkillRequest('push',oid,target))
        else:steps.extend((SkillRequest('grasp',oid),SkillRequest('place',oid,target)))
    return steps
