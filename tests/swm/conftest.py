"""Isolated fixtures only. No robot, camera, GPU or real physical claims."""
import copy
import hashlib
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.scene import SceneWorldModel,CheckpointSynchronizer
from rm75_app.swm.skills import AtomicSkillRuntime,PlannedSkill,PlanAudit,ExecutionReceipt,REQUIRED_AUDITS
from rm75_app.swm.scene import digest


def pose(x=0.,y=0.,z=0.,yaw=0.):
    T=np.eye(4);c,s=np.cos(yaw),np.sin(yaw);T[:2,:2]=[[c,-s],[s,c]];T[:3,3]=[x,y,z];return T.tolist()


class Clock:
    def __init__(self):self.t=100.
    def __call__(self):return self.t
    def tick(self,dt=.1):self.t+=dt;return self.t


class Source:
    def __init__(self,world,clock):
        self.world=world;self.clock=clock;self.seq=0;self.session='camera-session'
        self.poses={oid:pose(.3 if oid!='table' else 0.,z=.03 if oid!='table' else 0.) for oid in world._objects}
        self.holding='empty';self.boundaries=[];self.mutate=lambda x:None
    def capture(self,ids,after,boundary):
        self.clock.t=max(self.clock(),after);stamp=self.clock.tick();self.seq+=1;self.boundaries.append(boundary)
        rows=[dict(id=oid,T_world_object=copy.deepcopy(self.poses[oid]),mesh_sha256=self.world.assets[self.world._objects[oid]['asset_id']]['mesh_sha256'],
                   accepted=True,tracking_state='tracking',source='rrtrack_foundationpose',captured_at=stamp,sequence=self.seq,
                   position_uncertainty_m=.001,rotation_uncertainty_rad=.01) for oid in ids]
        batch=dict(schema='rm75_swm_observation_v1',world_frame='base_link',calibration_id='extrinsics-v1',
                   sensor_session=self.session,domain=self.world.domain,objects=rows,
                   robot=dict(joint_names=[f'joint_{i}' for i in range(1,8)],positions=[0.]*7,units='rad',
                              idle=True,captured_at=stamp,T_world_tcp=pose(.3,z=.2),holding=self.holding,
                              gripper_observed=True,holding_evidence='fixture-independent-evidence'))
        self.mutate(batch);return batch


class Mirror:
    def __init__(self):self.rows=[];self.fail=False
    def replace_scene(self,snapshot):
        self.rows.append(copy.deepcopy(snapshot))
        return 'wrong' if self.fail else snapshot['snapshot_id']


class Backend:
    execution_domain='fixture';capabilities=frozenset(('grasp','place','push','pull','rotate'))
    def __init__(self,source,clock):
        self.source=source;self.clock=clock;self.executions=0;self.plans=[]
        self.on_plan=lambda request,snapshot:None;self.on_execute=None;self.fail_audit=False
    def plan(self,request,snapshot):
        self.plans.append(copy.deepcopy(snapshot));self.on_plan(request,snapshot)
        target=pose(.3,z=.15) if request.skill=='grasp' else request.target
        return PlannedSkill(digest(request.as_dict()),snapshot['snapshot_id'],digest([snapshot['snapshot_id'],request.as_dict()]),
                            request,target,'fixture-native-solver')
    def audit(self,plan,snapshot):
        self.audited=snapshot
        return PlanAudit(plan.payload_digest,snapshot['snapshot_id'],frozenset() if self.fail_audit else REQUIRED_AUDITS)
    def execute(self,plan,audit):
        self.executions+=1;start=self.clock();self.clock.tick(.2);request=plan.payload
        self.source.poses[request.object_id]=copy.deepcopy(plan.expected_object_pose)
        if request.skill=='grasp':self.source.holding=request.object_id
        if request.skill=='place':self.source.holding='empty'
        if self.on_execute:self.on_execute(request)
        return ExecutionReceipt(True,start,self.clock(),True,'action-'+str(self.executions))


@pytest.fixture
def rig(tmp_path):
    path=tmp_path/'mesh.ply';path.write_text('fixture mesh bytes - not used by a physics engine')
    sha=hashlib.sha256(path.read_bytes()).hexdigest()
    asset=dict(source='legacy',units='m',metric_scale_verified=True,scale_evidence='test fixture',
               mesh_path=str(path),mesh_sha256=sha,collision_path=str(path),collision_sha256=sha,
               collision_kind='cuboid',collision_dimensions_m=[.05,.05,.05],volume_m3=.000125,
               functional_poses=[dict(id='side',skill=s,T_object_function=pose(.02),provenance='fixture') for s in ('grasp','pull','rotate')])
    manifest=dict(schema='rm75_swm_v1',world_frame='base_link',calibration_id='extrinsics-v1',observation_domain='fixture',
                  assets={'asset':asset},objects=[dict(id='a',name='block',asset_id='asset'),dict(id='b',name='block',asset_id='asset'),
                  dict(id='table',name='table',asset_id='asset',fixed=True)])
    world=SceneWorldModel(manifest);clock=Clock();source=Source(world,clock);mirror=Mirror();events=[]
    sync=CheckpointSynchronizer(world,source,mirror,clock=clock,emit=lambda **row:events.append(row))
    backend=Backend(source,clock);stop=SimpleNamespace(check=lambda:None)
    runtime=AtomicSkillRuntime(sync,backend,clock=clock,stop=stop,max_replans=2)
    return SimpleNamespace(world=world,clock=clock,source=source,mirror=mirror,sync=sync,backend=backend,runtime=runtime,
                           manifest=manifest,events=events,stop=stop)
