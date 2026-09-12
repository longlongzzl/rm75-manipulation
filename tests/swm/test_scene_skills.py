import copy
import numpy as np
import pytest
from .conftest import pose
from rm75_app.swm.scene import SceneWorldModel,ObservationUnavailable,SceneInvalid,transform,pose_error
from rm75_app.swm.skills import SkillRequest,SWMTools


def test_sync_full_scene_exact_mirror(rig):
    snap=rig.sync.sync('before_grasp');assert snap['valid'] and snap==rig.mirror.rows[-1]
    assert set(snap['objects'])=={'a','b','table'}
    snap['objects']['a']['measured']['T_world_object'][0][3]=99
    assert rig.world.snapshot()['objects']['a']['measured']['T_world_object'][0][3]==.3


@pytest.mark.parametrize('bad', ['missing','duplicate','lost','stale','future','mesh','frame','session','uncertain','robot_stale','units','holding','not_idle','unknown'])
def test_checkpoint_rejects_no_partial_commits(rig,bad):
    rig.sync.sync('initial');old=rig.world.snapshot()
    def mutate(data):
        if bad=='missing':data['objects'].pop()
        if bad=='duplicate':data['objects'].append(copy.deepcopy(data['objects'][0]))
        if bad=='lost':data['objects'][0]['accepted']=False
        if bad=='stale':data['objects'][0]['captured_at']-=10
        if bad=='future':data['objects'][0]['captured_at']+=10
        if bad=='mesh':data['objects'][0]['mesh_sha256']='0'*64
        if bad=='frame':data['world_frame']='camera'
        if bad=='session':data['sensor_session']='different'
        if bad=='uncertain':data['objects'][0]['position_uncertainty_m']=.1
        if bad=='robot_stale':data['robot']['captured_at']-=10
        if bad=='units':data['robot']['units']='degrees'
        if bad=='holding':data['robot']['gripper_observed']=False
        if bad=='not_idle':data['robot']['idle']=False
        if bad=='unknown':data['objects'][0]['id']='unregistered'
    rig.source.mutate=mutate
    with pytest.raises((ValueError,ObservationUnavailable)):rig.sync.sync('bad')
    snap=rig.world.snapshot();assert not snap['valid'] and snap['objects']==old['objects']
    assert len(rig.mirror.rows)==1


def test_scene_sink_failure_latches_invalid(rig):
    rig.mirror.fail=True
    with pytest.raises(SceneInvalid):rig.sync.sync('fail')
    assert not rig.world.valid
    rig.mirror.fail=False;assert rig.sync.sync('recover')['valid']


def test_sensor_restart_requires_explicit_reinitialization(rig):
    rig.sync.sync('first');rig.source.session='new-session'
    with pytest.raises(ObservationUnavailable):rig.sync.sync('implicit')
    rig.world.begin_new_sensor_session();assert rig.sync.sync('explicit')['sensor_session']=='new-session'


@pytest.mark.parametrize('bad', ['scale','reflect','nan','homogeneous'])
def test_bad_poses(bad):
    m=np.eye(4)
    if bad=='scale':m[0,0]=2
    if bad=='reflect':m[0,0]=-1
    if bad=='nan':m[0,3]=np.nan
    if bad=='homogeneous':m[3,0]=1
    with pytest.raises(ValueError):transform(m)


def test_grasp_place_are_two_verified_atoms_with_checkpoint_each(rig):
    grasp=rig.runtime.run(SkillRequest('grasp','a'));place=rig.runtime.run(SkillRequest('place','a',pose(.45,z=.03)))
    assert grasp.skill_verified and place.skill_verified and rig.backend.executions==2
    assert rig.source.boundaries==['before_grasp','pre_execute_grasp','after_grasp','before_place','pre_execute_place','after_place']
    assert rig.world.snapshot()['robot']['holding']=='empty'
    assert rig.world.snapshot()['objects']['a']['measured']['T_world_object']==pose(.45,z=.03)


def test_moved_obstacle_discards_plan_not_just_target(rig):
    def on_plan(req,snap):
        if len(rig.backend.plans)==1:rig.source.poses['b']=pose(.5,z=.03)
    rig.backend.on_plan=on_plan
    result=rig.runtime.run(SkillRequest('push','a',pose(.4,z=.03)))
    assert result.skill_verified and len(rig.backend.plans)==2 and rig.backend.executions==1
    assert result.attempts[0]['reason']=='scene_changed_during_planning'


def test_persistent_drift_never_executes(rig):
    rig.backend.on_plan=lambda req,snap:rig.source.poses.__setitem__('a',pose(.3+.02*len(rig.backend.plans),z=.03))
    result=rig.runtime.run(SkillRequest('push','a',pose(.5,z=.03)))
    assert not result.skill_verified and rig.backend.executions==0 and len(rig.backend.plans)==3


def test_wrong_placement_keeps_actual_pose_and_demands_new_grasp(rig):
    rig.source.holding='a'
    rig.backend.on_execute=lambda req:rig.source.poses.__setitem__('a',pose(.6,z=.03))
    out=rig.runtime.run(SkillRequest('place','a',pose(.4,z=.03)))
    assert out.status=='replan_required' and not out.skill_verified and rig.backend.executions==1
    assert rig.world.snapshot()['objects']['a']['measured']['T_world_object']==pose(.6,z=.03)


def test_close_command_is_not_grasp_success(rig):
    rig.backend.on_execute=lambda req:setattr(rig.source,'holding','empty')
    out=rig.runtime.run(SkillRequest('grasp','a'))
    assert not out.skill_verified and rig.backend.executions==3


def test_audit_failure_blocks_execution(rig):
    rig.backend.fail_audit=True
    with pytest.raises(SceneInvalid):rig.runtime.run(SkillRequest('grasp','a'))
    assert not rig.backend.executions


def test_grasp_failure_no_downstream_place(rig):
    rig.source.holding='b'
    with pytest.raises(SceneInvalid):rig.runtime.run(SkillRequest('grasp','a'))
    assert not rig.backend.executions


def test_unknown_execution_error_not_swallowed(rig):
    def fail(*args):raise RuntimeError('unclassified SDK error')
    rig.backend.execute=fail
    with pytest.raises(RuntimeError,match='unclassified'):rig.runtime.run(SkillRequest('grasp','a'))
    assert len(rig.backend.plans)==1


@pytest.mark.parametrize('skill', ['pull','rotate'])
def test_other_atoms_need_real_functional_binding(rig,skill):
    with pytest.raises(ValueError):rig.runtime.run(SkillRequest(skill,'a',pose(.4,z=.03)))
    out=rig.runtime.run(SkillRequest(skill,'a',pose(.4,z=.03),functional_pose_id='side'))
    assert out.skill_verified


def test_get_functional_pose_is_object_relative_not_stale(rig):
    rig.sync.sync('first');tool=SWMTools(rig.world)
    result=tool.get_functional_poses('a','grasp')[0]
    assert result['T_world_function'][0][3]==pytest.approx(.32)
    rig.source.poses['a']=pose(.5,z=.03);rig.sync.sync('moved')
    assert tool.get_functional_poses('a','grasp')[0]['T_world_function'][0][3]==pytest.approx(.52)
    rig.world.invalidate('lost')
    with pytest.raises(SceneInvalid):tool.get_object_pose('a')


def test_no_coordinate_query_for_unobserved_scene(rig):
    with pytest.raises(SceneInvalid):SWMTools(rig.world).get_object_pose('a')


def test_execution_mode_not_inferred_from_labels(rig):
    from rm75_app.swm.skills import AtomicSkillRuntime
    rig.backend.execution_domain='real'
    with pytest.raises(PermissionError):AtomicSkillRuntime(rig.sync,rig.backend,clock=rig.clock,stop=rig.stop)


def test_unavailable_skill_is_not_fake_success(rig):
    rig.backend.capabilities=frozenset(['grasp'])
    with pytest.raises(NotImplementedError):rig.runtime.run(SkillRequest('push','a',pose(.4)))


def test_do_not_loosen_task_tolerance(rig):
    with pytest.raises(ValueError):rig.runtime.run(SkillRequest('push','a',pose(.4),position_tolerance_m=.02))


def test_metric_and_model_required(rig):
    m=copy.deepcopy(rig.manifest);m['assets']['asset']['metric_scale_verified']=False
    with pytest.raises(ValueError):SceneWorldModel(m)
    m=copy.deepcopy(rig.manifest);m['assets']['asset']['mesh_sha256']='short'
    with pytest.raises(ValueError):SceneWorldModel(m)
