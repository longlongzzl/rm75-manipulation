import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from .conftest import pose
from rm75_app.swm.scene import SceneInvalid,ObservationUnavailable,digest
from rm75_app.swm.perception import RRTrackCheckpointSource,FoundationPoseCheckpointEstimator,SAM3DAssetBuilder
from rm75_app.swm.adapters import compose_task,TransactionalSceneMirror,NativeAtomicBinding,NativeAtomicBackend
from rm75_app.swm.skills import SkillRequest,PlannedSkill,PlanAudit,REQUIRED_AUDITS
from rm75_app.swm.integration import dispatch_if_enabled,contract_status
from rm75_app.swm.planning import ParallelHypothesisPlanner


def test_rrtrack_bridge_uses_original_pose_timestamp_and_identity(rig):
    binding=dict(asset_name='asset',mesh_sha256=rig.world.assets['asset']['mesh_sha256'],position_uncertainty_m=.001,rotation_uncertainty_rad=.01)
    output=SimpleNamespace(T_cam_obj=pose(.1),frame_index=12,state='tracking',accepted=True)
    sample=SimpleNamespace(instance_id='a',asset_name='asset',timestamp_s=42.,output=output)
    port=RRTrackCheckpointSource(lambda ids,**kw:dict(samples=[sample],sensor_session='s',robot={}),{'a':binding},pose(.2),'cal')
    batch=port.capture(['a'],after=40,boundary='before_grasp')
    assert batch['objects'][0]['captured_at']==42 and batch['objects'][0]['sequence']==12
    assert batch['objects'][0]['T_world_object'][0][3]==pytest.approx(.3)
    output.accepted=False
    with pytest.raises(ObservationUnavailable):port.capture(['a'],after=40,boundary='before_grasp')


def test_fp_reregisters_after_gap_or_quality_failure():
    calls=[]
    class Est:
        def register(self,**kw):calls.append('register');return pose()
        def track_one(self,**kw):calls.append('track');return pose()
    quality={'accepted':True}
    port=FoundationPoseCheckpointEstimator(Est(),lambda *a,**k:quality)
    data=dict(rgb=np.zeros((3,3,3),dtype=np.uint8),depth_m=np.ones((3,3)),mask=np.ones((3,3),bool),K=np.eye(3))
    port.estimate(**data,captured_at=1);port.estimate(**data,captured_at=1.2);port.estimate(**data,captured_at=3)
    assert calls==['register','track','register']
    quality['accepted']=False
    with pytest.raises(ObservationUnavailable):port.estimate(**data,captured_at=3.1)
    quality['accepted']=True;port.estimate(**data,captured_at=3.2)
    assert calls[-1]=='register'


def test_sam3d_requires_metric_and_real_triangle_proxy(tmp_path):
    import trimesh
    inputs=[]
    mesh=trimesh.creation.box(extents=[1,1,1])
    builder=SAM3DAssetBuilder(lambda *a,**kw:inputs.append(kw) or {'output':1},lambda raw:mesh)
    transform=np.eye(4);transform[:3,:3]*=.05
    asset=builder.build(image=np.zeros((2,2,3)),mask=np.ones((2,2)),asset_id='new_box',output_dir=tmp_path/'model',
        model_to_metric=transform,scale_evidence='measured 50mm cube',collision_builder=lambda m:m)
    assert asset['source']=='sam3d' and asset['volume_m3']==pytest.approx(.05**3)
    assert inputs[0]['seed']==42 and asset['metric_scale_verified']
    assert Path(asset['collision_path']).is_file()
    with pytest.raises(ValueError):builder.build(image=None,mask=None,asset_id='x',output_dir=tmp_path/'bad',model_to_metric=np.eye(4),
        scale_evidence='',collision_builder=lambda m:m)


def test_mirror_synchronizes_both_planner_and_simulator_without_motion(rig):
    snapshot=rig.sync.sync('capture');calls=[]
    def apply(s):calls.append(s);return s['snapshot_id']
    port=SimpleNamespace(apply_idle_snapshot=apply)
    mirror=TransactionalSceneMirror(port,port)
    assert mirror.replace_scene(snapshot)==snapshot['snapshot_id'] and len(calls)==2
    failed=SimpleNamespace(apply_idle_snapshot=lambda s:'different')
    mirror=TransactionalSceneMirror(port,failed)
    with pytest.raises(SceneInvalid):mirror.replace_scene(snapshot)
    assert mirror.valid is False


def test_all_scenarios_use_same_atomic_types():
    targets=[('a',pose(.4)),('b',pose(.5))]
    assert [s.skill for s in compose_task('pickplace',targets)]==['grasp','place','grasp','place']
    assert [s.skill for s in compose_task('magnetic',targets)]==['grasp','place','grasp','place']
    assert [s.skill for s in compose_task('pusht',targets[:1])]==['push']


def test_default_legacy_is_not_claimed_to_be_atomic():
    assert dispatch_if_enabled({'task':'pickplace','mode':'sim'},{},None,None,None,None) is None
    status=contract_status({})
    assert not status['enabled'] and not status['native_atomic_integration_verified']
    for task in ('pickplace','magnetic','pusht'):
        with pytest.raises(RuntimeError,match='SWM_ATOMIC_ADAPTER_REQUIRED'):
            dispatch_if_enabled({'task':task,'mode':'sim'},{'swm':{'enabled':True}},None,None,None,None)


@pytest.mark.parametrize('value',[0,1,'false',None,[]])
def test_bad_flag_not_silently_ignored(value):
    with pytest.raises(ValueError):dispatch_if_enabled({}, {'swm':{'enabled':value}},None,None,None,None)


def test_worker_reaches_swm_after_isolation_and_lease_not_via_browser_code():
    root=Path(__file__).resolve().parents[2]
    path=root/'rm75_app/workcell/worker.py'
    if not path.is_file():path=Path(__file__).resolve().parents[3]/'rm75_app/workcell/worker.py'
    source=path.read_text()
    assert source.index('isolate_task_worker(spec,events)') < source.index('with ResourceLease(')<source.index('swm_result=dispatch_if_enabled')
    ast.parse(source)


def test_parallel_candidates_cross_check_all_parameters(rig):
    snapshot=rig.sync.sync('before_grasp');request=SkillRequest('grasp','a');closed=[]
    class Solver:
        owns_real_executor=False
        def solve(self,req,snap,params):
            return PlannedSkill(digest(req.as_dict()),snap['snapshot_id'],str(params['density_kg_m3']),{},pose(.3,z=.15),'native-fixture')
        def evaluate(self,plan,snap,params):
            return dict(snapshot_id=snap['snapshot_id'],payload_digest=plan.payload_digest,
                        feasible=plan.payload_digest=='1000',cost=1.)
        def close(self):closed.append(1)
    hs=[dict(id='a',parameters=dict(static_friction=.5,dynamic_friction=.3,density_kg_m3=1000)),
        dict(id='b',parameters=dict(static_friction=.6,dynamic_friction=.4,density_kg_m3=1500))]
    plan,evidence=ParallelHypothesisPlanner(Solver,workers=2).solve(request,snapshot,hs)
    assert plan.payload_digest=='1000' and evidence['required_hypotheses']==['a','b'] and len(closed)==6


def test_native_binding_does_not_manufacture_capabilities(rig):
    backend=NativeAtomicBackend({'grasp':NativeAtomicBinding(rig.backend.plan,rig.backend.audit,rig.backend.execute,'original_grasp_only')},execution_domain='fixture')
    assert backend.capabilities==frozenset(['grasp'])
