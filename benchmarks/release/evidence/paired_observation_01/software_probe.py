import copy,json,sys,tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path.cwd()));sys.path.insert(0,str(Path.cwd()/'tests/swm'))
from conftest import rig,pose
from rm75_app.swm.scene import digest
from rm75_app.swm.skills import SkillRequest,PlannedSkill,PlanAudit,REQUIRED_AUDITS,ExecutionReceipt
from rm75_app.swm.native_skills import NativePrimitive,NativeStage,NativeStageState
from rm75_app.swm.paired_grasp_place import PairedGraspPlace,atomic_groups
from rm75_app.planning.contracts import JointTrajectory

def run(mode):
    with tempfile.TemporaryDirectory() as folder:
        r=rig.__wrapped__(Path(folder));events=[];held=[False];saved=[None];calls=[]
        jaw={f'gripper_{side}_{part}_Joint':.6 for side in ('Left','Right') for part in ('1','2','Support')}
        r.source.mutate=lambda batch:batch['robot'].update(gripper_positions=jaw)
        capture=r.source.capture
        def guarded(*args,**kwargs):
            assert not held[0],'Object capture attempted during holding'
            events.append(('capture',kwargs['boundary'],r.clock()))
            return capture(*args,**kwargs)
        r.source.capture=guarded
        target=pose(.4,z=.03);g=SkillRequest('grasp','a');p=SkillRequest('place','a',target=target)
        sink=SimpleNamespace(measured_lift_audit=None)
        def feedback():return dict(source='measured_feedback',idle=True,positions=[0.]*7,
            joint_names=[f'joint_{i}' for i in range(1,8)],gripper_positions=jaw,T_world_tcp=pose(.3,z=.2),captured_at=r.clock())
        sink.feedback=feedback
        def audit(plan,snapshot,**kwargs):
            if kwargs:assert kwargs['execution_state']['source']=='approved_grasp_place_prediction'
            return PlanAudit(plan.payload_digest,snapshot['snapshot_id'],REQUIRED_AUDITS)
        class Phases:
            def plan_pair(self,grasp,place,snapshot):
                empty=NativeStageState(None,'empty',gripper_positions=jaw)
                carry=NativeStageState(True,'a',np.eye(4));released=NativeStageState(False,'empty',released_object_pose=target)
                path=JointTrajectory(tuple(snapshot['robot']['joint_names']),np.zeros((2,7)),dt=np.array([.1]))
                gs=(NativeStage('approach',path,state_before=empty,state_after=empty),
                    NativeStage('grasp',path,True,empty,carry,('a',)),
                    NativeStage('lift',path,state_before=carry,state_after=carry,allow_start_contact_escape=True))
                ps=(NativeStage('preplace',path,state_before=carry,state_after=carry),
                    NativeStage('place',path,False,carry,released),NativeStage('retreat',path,state_before=released,state_after=released))
                def plan(request,stages,expected):
                    primitive=NativePrimitive(request.skill,'a',stages)
                    return PlannedSkill(digest(request.as_dict()),snapshot['snapshot_id'],primitive.fingerprint(),primitive,expected,'software_fixture')
                return plan(grasp,gs,pose(.3,z=.2)),plan(place,ps,target)
            def execute(self,plan,certificate):
                assert certificate.payload_digest==plan.payload_digest
                start=r.clock();calls.append(plan.payload.skill)
                if plan.payload.skill=='grasp':
                    saved[0]=copy.deepcopy(r.world.snapshot()['objects'])
                    held[0]=True;events.append(('close',r.clock()))
                    if mode=='cancel':raise InterruptedError('fixture cancellation after close')
                    sink.measured_lift_audit('lift',plan.payload.stages[-1].trajectory)
                    assert r.world.snapshot()['objects']==saved[0]
                    r.clock.tick(.1)
                else:
                    assert held[0] and r.world.snapshot()['objects']==saved[0]
                    events.append(('release',r.clock.tick(.1)))
                    events.append(('retreat_completed',r.clock.tick(.1)));held[0]=False
                    r.source.poses['a']=pose(.45 if mode=='wrong' else .4,z=.03)
                return ExecutionReceipt(True,start,r.clock(),True,'fixture-'+str(len(calls)))
        phases=Phases();phases.audit=audit
        r.runtime.grasp_place_policy=PairedGraspPlace(phases,audit,sink)
        try:groups=list(atomic_groups(r.runtime,[g,p]))
        except InterruptedError:
            assert mode=='cancel' and calls==['grasp'] and not r.world.valid
            assert 'after_release_retreat' not in r.source.boundaries
            return dict(case=mode,passed=True,subsequent_place_executed=False,final_observation=False)
        assert len(groups)==1 and len(groups[0])==2 and calls==['grasp','place']
        first,last=[result for request,result in groups[0]]
        assert first.status=='visual_verification_pending' and not first.skill_verified
        assert last.skill_verified==(mode=='success')
        assert r.source.boundaries==['before_grasp','pre_execute_grasp','after_release_retreat']
        final_capture=next(i for i,row in enumerate(events) if row[:2]==('capture','after_release_retreat'))
        assert next(i for i,row in enumerate(events) if row[0]=='release')<next(i for i,row in enumerate(events) if row[0]=='retreat_completed')<final_capture
        assert last.status==('verified' if mode=='success' else 'replan_required')
        return dict(case=mode,passed=True,boundaries=r.source.boundaries,grasp_visually_verified=False,
            place_verified=last.skill_verified,held_measured_rows_unchanged=True,events=events)
print(json.dumps(dict(domain='software_fixture_not_native_simulation',results=[run(m) for m in ('success','wrong','cancel')]),ensure_ascii=False))
