"""One observation window around two original, jointly audited atomic skills.

Measured object rows remain untouched while carrying. This is not a legacy
episode: both original primitives, receipts and stage audits remain explicit.
"""
from dataclasses import replace
import uuid
import numpy as np
from .scene import SceneInvalid,digest,pose_error,moved_between
from .skills import PlannedSkill,PlanAudit,SkillResult,SkillVerification,REQUIRED_AUDITS
from .native_skills import NativePrimitive


def reaudited_prediction(plan,snapshot,feedback,auditor):
    """Adjust only the joint start and observed jaw geometry, never an object pose."""
    if feedback.get('source')!='measured_feedback' or feedback.get('idle') is not True:
        raise SceneInvalid('Fresh idle nonvisual execution feedback required')
    q=np.asarray(feedback['positions'],float);stages=list(plan.payload.stages)
    old=np.asarray(stages[0].trajectory.positions,float)
    if q.shape!=(7,) or not np.isfinite(q).all() or np.max(np.abs(q-old[0]))>.02:
        raise SceneInvalid('Nonvisual start exceeds original endpoint allowance')
    path=old.copy();path[0]=q
    stages[0]=replace(stages[0],trajectory=replace(stages[0].trajectory,positions=path))
    def jaw(state):
        return (replace(state,gripper_closed=None,gripper_positions=feedback['gripper_positions'])
                if state.holding!='empty' else state)
    stages=tuple(replace(stage,state_before=jaw(stage.state_before),state_after=jaw(stage.state_after)) for stage in stages)
    primitive=replace(plan.payload,stages=stages)
    updated=replace(plan,payload=primitive,payload_digest=primitive.fingerprint())
    audit=auditor(updated,snapshot,execution_state=dict(source='approved_grasp_place_prediction',
        snapshot_id=snapshot['snapshot_id'],payload_digest=updated.payload_digest,
        feedback=feedback,state=stages[0].state_before))
    if (audit.payload_digest!=updated.payload_digest or audit.snapshot_id!=snapshot['snapshot_id']
            or not REQUIRED_AUDITS<=set(audit.passed)):
        raise SceneInvalid('Complete nonvisual remaining-path audit required')
    return updated,audit


class PredictedLiftAudit:
    def __init__(self,plan,snapshot,auditor,feedback,stop,emit):
        self.plan=plan;self.snapshot=snapshot;self.auditor=auditor
        self.feedback=feedback;self.stop=stop;self.emit=emit;self.used=False
    def __call__(self,stage,trajectory):
        self.stop.check()
        rows=[row for row in self.plan.payload.stages if row.name=='lift']
        if (self.used or stage!='lift' or len(rows)!=1
                or digest(np.asarray(rows[0].trajectory.positions).tolist())!=digest(np.asarray(trajectory.positions).tolist())):
            raise SceneInvalid('Only the approved paired lift can use predicted attachment')
        self.used=True
        primitive=replace(self.plan.payload,stages=(rows[0],))
        plan=replace(self.plan,payload=primitive,payload_digest=primitive.fingerprint())
        updated,audit=reaudited_prediction(plan,self.snapshot,self.feedback(),self.auditor)
        self.emit(kind='swm_predicted_attachment_lift_audited',payload_digest=updated.payload_digest,
            snapshot_id=audit.snapshot_id,attachment_source='simulation_prediction',object_observation=False,
            skill_verified=False)
        return updated.payload.stages[0].trajectory


class PairedGraspPlace:
    def __init__(self,phases,auditor,sink):
        self.phases=phases;self.auditor=auditor;self.sink=sink

    def run(self,runtime,grasp_request,place_request):
        if (grasp_request.skill!='grasp' or place_request.skill!='place'
                or grasp_request.object_id!=place_request.object_id):
            raise SceneInvalid('Paired place of the same instance required before grasp')
        if not runtime._execution_lock.acquire(blocking=False):raise RuntimeError('Atomic runtime is busy')
        sync=runtime.sync;world=runtime.world;operation=uuid.uuid4().hex
        try:
            for attempt in range(runtime.max_replans+1):
                runtime.stop.check();initial=sync.sync('before_grasp')
                grasp=grasp_request.resolve(initial);place=place_request.resolve(initial)
                runtime._preconditions(grasp,initial)
                if place.position_tolerance_m>runtime.max_position_tolerance or place.rotation_tolerance_rad>runtime.max_rotation_tolerance:
                    raise SceneInvalid('Paired place may not loosen goal tolerances')
                gp,pp=self.phases.plan_pair(grasp,place,initial)
                current=sync.sync('pre_execute_grasp')
                resolved=place_request.resolve(current)
                p,r=pose_error(place.target,resolved.target)
                if moved_between(initial,current,sync.policy) or p>place.position_tolerance_m or r>place.rotation_tolerance_rad:
                    continue
                for request,plan in ((grasp,gp),(place,pp)):
                    if plan.skill_digest!=digest(request.as_dict()) or plan.source_snapshot_id!=initial['snapshot_id']:
                        raise SceneInvalid('Joint plan identity mismatch')
                combined=NativePrimitive('grasp',grasp.object_id,gp.payload.stages+pp.payload.stages)
                joint=PlannedSkill(digest([grasp.as_dict(),place.as_dict()]),current['snapshot_id'],
                    combined.fingerprint(),combined,pp.expected_object_pose,'joint_original_grasp_place')
                audit=self.phases.audit(joint,current)
                if (audit.payload_digest!=joint.payload_digest or audit.snapshot_id!=current['snapshot_id']
                        or not REQUIRED_AUDITS<=set(audit.passed)):
                    raise SceneInvalid('Complete joint grasp-place audit required')
                if (world.snapshot()['snapshot_id']!=current['snapshot_id'] or
                        runtime.clock()-min(current['robot']['captured_at'],*[o['measured']['captured_at'] for o in current['objects'].values()])>sync.policy.max_age_s):
                    continue
                break
            else:raise SceneInvalid('Fresh empty-hand joint planning budget exhausted')
            runtime.stop.check()
            world.begin_predicted_execution(operation,current['snapshot_id'],grasp.object_id,gp.expected_object_pose)
            self.sink.measured_lift_audit=PredictedLiftAudit(gp,current,self.auditor,self.sink.feedback,runtime.stop,sync.emit)
            # Every grasp stage is a prefix of the unchanged fully audited joint
            # primitive. This is a scoped proof projection, not new audit tags.
            if combined.fingerprint()!=audit.payload_digest or gp.payload.fingerprint()!=gp.payload_digest:
                raise SceneInvalid('Joint audit changed before atomic execution')
            grasp_audit=PlanAudit(gp.payload_digest,current['snapshot_id'],audit.passed)
            gr=self.phases.execute(gp,grasp_audit)
            self._receipt(runtime,gr,current['robot']['captured_at'])
            pending=SkillResult('grasp',grasp.object_id,'visual_verification_pending',True,False,False,
                world.snapshot()['snapshot_id'],[dict(operation_id=operation,actual_action_id=gr.actual_action_id,
                joint_audit_digest=audit.payload_digest,attachment_source='simulation_prediction',
                visual_verification_pending=True)],world.domain)
            sync.emit(kind='swm_grasp_visual_verification_pending',operation_id=operation,
                object_id=grasp.object_id,command_success=True,skill_verified=False)
            runtime.stop.check()
            placed,place_audit=reaudited_prediction(pp,current,self.sink.feedback(),self.auditor)
            pr=self.phases.execute(placed,place_audit)
            self._receipt(runtime,pr,gr.ended_at)
            world.finish_predicted_execution(operation,placed.expected_object_pose)
            final=sync.sync('after_release_retreat',after=pr.ended_at)
            target=place_request.resolve(final)
            actual=final['objects'][place.object_id]['measured']['T_world_object']
            p,r=pose_error(actual,target.target)
            reached=p<=place.position_tolerance_m and r<=place.rotation_tolerance_rad
            if place.goal_predicate=='native_relation':
                if runtime.goal_verifier is None:raise SceneInvalid('Native final relation verifier required')
                verified=runtime.goal_verifier(place_request,final)
                if (not isinstance(verified,SkillVerification) or verified.request_digest!=digest(place_request.as_dict())
                        or verified.snapshot_id!=final['snapshot_id']):raise SceneInvalid('Final relation evidence mismatch')
                reached=verified.reached
            reached=bool(reached and final['robot']['holding']=='empty')
            result=SkillResult('place',place.object_id,'verified' if reached else 'replan_required',True,reached,reached,
                final['snapshot_id'],[dict(operation_id=operation,actual_action_id=pr.actual_action_id,
                    goal_error_m=p,goal_error_rad=r,verification_boundary='after_release_retreat')],world.domain,
                '' if reached else 'Released outcome differs; plan a new grasp, never replay empty-hand place')
            return [(grasp_request,pending),(place_request,result)]
        except BaseException as exc:
            world.invalidate('paired_'+type(exc).__name__)
            raise
        finally:
            self.sink.measured_lift_audit=None
            runtime._execution_lock.release()

    @staticmethod
    def _receipt(runtime,receipt,after):
        if (receipt.command_success is not True or receipt.motion_known_complete is not True
                or not receipt.actual_action_id or receipt.actual_action_id in runtime._used_actions
                or not after<=receipt.started_at<=receipt.ended_at<=runtime.clock()):
            raise SceneInvalid('Unknown or failed paired execution; no automatic continuation')
        runtime._used_actions.add(receipt.actual_action_id)


def atomic_groups(runtime,requests):
    """Consumers accept pending grasp only inside the trusted completed pair."""
    requests=list(requests)
    if not 1<=len(requests)<=64:raise SceneInvalid('Bounded nonempty atomic sequence required')
    index=0;policy=getattr(runtime,'grasp_place_policy',None)
    while index<len(requests):
        request=requests[index]
        if policy is not None and request.skill in ('grasp','place'):
            if not isinstance(policy,PairedGraspPlace) or index+1>=len(requests):
                raise SceneInvalid('Complete trusted grasp-place context required')
            group=policy.run(runtime,request,requests[index+1]);index+=2
        else:
            group=[(request,runtime.run(request))];index+=1
        yield group
        if not group[-1][1].skill_verified:return
