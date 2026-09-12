"""Rollout status and explicit SWM migration boundary for the existing workcell.

A listing of grasp/place/push/pull/rotate is not evidence of native integration.
Existing production algorithms are retained. SWM enforcement may only be enabled
once a trusted per-task factory returns the shared checkpointed atomic runtime.
There is NO fallback to a legacy macro after SWM execution/measurement fails.
"""
from __future__ import annotations
import copy
from contextlib import AbstractContextManager, ExitStack
from dataclasses import dataclass
import json
from pathlib import Path
from .scene import SceneWorldModel
from .skills import SKILLS, AtomicSkillRuntime

_FACTORIES={}


@dataclass(frozen=True)
class RuntimeSession:
    runtime: AtomicSkillRuntime
    requests: object
    goals_check: object

    def __post_init__(self):
        if not isinstance(self.runtime,AtomicSkillRuntime) or not callable(self.goals_check):
            raise TypeError('Session requires atomic runtime and independent goal verification')


class RuntimeContext(AbstractContextManager):
    """Factory-owned resource lifetime, including partially failed initialization.

    The trusted builder immediately registers each acquired resource with the
    supplied ExitStack. The worker owns this context; the web parent does not.
    Neither entry nor cleanup grants permission to connect real hardware.
    """
    def __init__(self, build):
        self.build=build
        self.resources=ExitStack()
        self.entered=False

    def __enter__(self):
        if self.entered:raise RuntimeError('Runtime context cannot be reused')
        self.entered=True
        try:
            session=self.build(self.resources)
            if not isinstance(session,RuntimeSession):raise TypeError('Factory must build RuntimeSession')
            return session
        except BaseException:
            self.resources.close()
            raise

    def __exit__(self, *exc):
        return self.resources.__exit__(*exc)


def register_runtime_factory(task, factory):
    """Application startup only; never invoke with browser/LLM-provided callables."""
    if task not in ('pickplace','magnetic','pusht') or not callable(factory):raise ValueError('Invalid runtime binding')
    if task in _FACTORIES:raise ValueError('Do not replace an active SWM adapter implicitly')
    _FACTORIES[task]=factory


def contract_status(profile):
    configured=profile.get('swm',{})
    return dict(schema='rm75_swm_contract_status_v1',architecture='SceneWorldModel + checkpointed atomic skills',
        synchronization='skill_boundary_not_realtime',pose_source='shared_RRTrack/FoundationPose',
        enabled=configured.get('enabled',False) is True,
        registered_tasks=sorted(_FACTORIES),skill_contracts=list(SKILLS),
        native_atomic_integration_verified=False,hardware_qualified=False,
        physics_identification='hypothesis_replay_of_same_measured_action; not response_gain_fit',
        required_boundaries={'pickplace':['before_grasp','after_grasp','before_place','after_place'],
            'magnetic':['before_grasp','after_grasp','before_place','after_place','verify_structure'],
            'pusht':['before_push','after_push']})


def dispatch_if_enabled(spec,profile,app_root,run_dir,stop,events):
    """Called under the existing workcell lease and AFTER existing authorization.

    None means the explicit pre-SWM legacy route. It does NOT mean checkpoint
    verification passed. A configured SWM route without native bindings fails.
    """
    options=profile.get('swm',{})
    if not isinstance(options,dict) or type(options.get('enabled',False)) is not bool:
        raise ValueError('swm.enabled must be a boolean')
    if not options.get('enabled',False):return None
    task=spec['task']
    if task not in _FACTORIES:
        raise RuntimeError('SWM_ATOMIC_ADAPTER_REQUIRED: retain the legacy mode for existing demos; do not claim checkpointed execution')
    if spec['mode']=='preview':raise RuntimeError('SWM preview uses compile-only APIs, not an execution factory')
    context=_FACTORIES[task](spec,profile,app_root,run_dir,stop,events)
    if not isinstance(context,AbstractContextManager):
        raise TypeError('Native factory must return a managed runtime context')
    with context as session:
        if not isinstance(session,RuntimeSession):raise TypeError('Managed factory must yield RuntimeSession')
        return _dispatch_session(spec,session,stop)


def _dispatch_session(spec,session,stop):
    runtime,requests,goals_check=session.runtime,session.requests,session.goals_check
    task=spec['task']
    expected='real' if spec['mode']=='real' else 'physics'
    if runtime.world.domain!=expected:raise PermissionError('SWM execution domain is not the requested task mode')
    outcomes=[]
    for request in requests:
        stop.check();result=runtime.run(request);outcomes.append(result.as_dict())
        if not result.skill_verified:
            return dict(command_success=result.command_success,task_success=False,
                        verification='swm_'+expected,swm_checkpointed=True,atomic_results=outcomes,
                        status='swm_atomic_replan_required',failure_skill=request.skill)
    final=runtime.sync.sync('verify_structure' if task=='magnetic' else 'verify_task')
    verified=goals_check(final)
    if type(verified) is not bool:raise TypeError('Task verification must return an explicit boolean')
    return dict(command_success=True,task_success=verified,verification='swm_'+expected,
                swm_checkpointed=True,atomic_results=outcomes,final_swm_snapshot=final,
                hardware_qualified=False)
