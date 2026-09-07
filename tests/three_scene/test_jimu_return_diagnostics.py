import copy
import threading
from types import SimpleNamespace as NS
import numpy as np
import pytest
from rm75_app.workcell.jimu_return_diagnostics import install_return_diagnostics,install_release_execution_observer
from rm75_app.workcell.world_only_contact import WorldOnlyContactUnsupported


def fixture(*, success=False):
    result=NS(success=success,status='Success' if success else 'TRAJOPT_FAIL')
    planner=NS(robot_cfg_dict={'kinematics':{'lock_joints':{'finger_joint':.6}}},
        attached_object_active=False,_disabled_collision_links=set(),_disabled_world_obstacles=set(),
        motion_gen=NS(rollout_fn=NS(primitive_collision_constraint=NS(enabled=True),
                                  robot_self_collision_constraint=NS(enabled=True))),
        _world=NS(objects=[NS(name='caller_world')]))
    checks=[];queries=[]
    def check(q):
        checks.append(np.asarray(q).tolist())
        return bool(q[0]==0),'VALID' if q[0]==0 else 'WORLD_COLLISION'
    planner.check_start_state=check
    direct=NS(_CUROBO_GPU_LOCK=threading.RLock(),_current_source_object_name=lambda args:'back_second_wall')
    def query(planner, start, goal, **kwargs):
        queries.append(kwargs.copy())
        return result
    def returning(planner,demo,args,start,goal,*,label,**kwargs):
        old=planner._world
        planner._world=NS(objects=[NS(name='refreshed_world')])
        try:return direct._profile_plan_to_joint_state(planner,start,goal,**kwargs)
        finally:planner._world=old
    direct._profile_plan_to_joint_state=query
    direct._plan_return_to_start_joint_curobo=returning
    portable=NS(direct=direct,
        _jimu_robot_world_obstacle_contacts=lambda p,q:[],
        _jimu_robot_internal_cube_contacts=lambda p,q:[],
        _jimu_curobo_raw_world_collision_snapshot=lambda p,q:{'nonzero':[]})
    demo=NS(active_joint_names=['finger_joint'],robot=NS(get_qpos=lambda:np.array([.6])))
    return planner,portable,demo,result,checks,queries


@pytest.mark.parametrize('success',[False,True])
def test_observe_only_failed_return_in_refreshed_world_and_preserve_native_result(success):
    planner,p,demo,result,checks,queries=fixture(success=success)
    rows=[];world=planner._world;locks=copy.deepcopy(planner.robot_cfg_dict)
    install_return_diagnostics(p,rows.append)
    actual=p.direct._plan_return_to_start_joint_curobo(planner,demo,NS(),np.zeros(7),np.ones(7),
        label='pre_release_return_check',num_trajopt_seeds=8,timeout=12.)
    assert actual is result and planner._world is world and planner.robot_cfg_dict==locks
    assert queries==[{'num_trajopt_seeds':8,'timeout':12.}]
    if success:
        assert rows==checks==[]
    else:
        row=rows[0]
        assert row['state_unchanged'] and row['diagnostic_complete']
        assert row['world_objects'][0]['name']=='refreshed_world'
        assert row['endpoints']['start']['valid'] and not row['endpoints']['goal']['valid']
        assert len(checks)==2 and row['gripper_lock_joints']=={'finger_joint':.6}


def test_diagnostic_is_bounded_and_never_leaks_to_unrelated_query():
    planner,p,demo,result,checks,_=fixture();rows=[]
    install_return_diagnostics(p,rows.append)
    p.direct._profile_plan_to_joint_state(planner,np.zeros(7),np.ones(7))
    assert not rows
    for _ in range(2):
        p.direct._plan_return_to_start_joint_curobo(planner,demo,NS(),np.zeros(7),np.ones(7),label='return')
    p.direct._profile_plan_to_joint_state(planner,np.zeros(7),np.ones(7))
    assert len(rows)==1 and len(checks)==2


def test_native_exception_is_not_suppressed_or_marked_as_diagnostic_success():
    planner,p,demo,_,_,_=fixture();rows=[]
    def fail(*args,**kwargs):raise RuntimeError('native solver error')
    p.direct._profile_plan_to_joint_state=fail
    install_return_diagnostics(p,rows.append)
    with pytest.raises(RuntimeError,match='native solver error'):
        p.direct._plan_return_to_start_joint_curobo(planner,demo,NS(),[0]*7,[1]*7,label='return')
    assert rows==[]


def test_real_mode_does_not_invoke_diagnostic():
    planner,p,demo,result,checks,_=fixture();rows=[]
    install_return_diagnostics(p,rows.append)
    actual=p.direct._plan_return_to_start_joint_curobo(planner,demo,NS(execute_real=True),[0]*7,[1]*7,label='return')
    assert actual is result and rows==checks==[]


def test_missing_geometry_evidence_stays_incomplete_without_changing_failed_result():
    planner,p,demo,result,_,_=fixture();rows=[]
    del p._jimu_robot_internal_cube_contacts
    install_return_diagnostics(p,rows.append)
    assert p.direct._plan_return_to_start_joint_curobo(planner,demo,NS(),[0]*7,[1]*7,label='return') is result
    assert rows[0]['state_unchanged'] and not rows[0]['diagnostic_complete']


def test_unexpected_diagnostic_collision_mutation_fails_closed():
    planner,p,demo,_,_,_=fixture();rows=[]
    def mutate(planner,q):
        planner._disabled_world_obstacles.add('table');return []
    p._jimu_robot_internal_cube_contacts=mutate
    install_return_diagnostics(p,rows.append)
    with pytest.raises(WorldOnlyContactUnsupported,match='changed native collision state'):
        p.direct._plan_return_to_start_joint_curobo(planner,demo,NS(),[0]*7,[1]*7,label='return')
    assert rows[0]['state_unchanged'] is False


@pytest.mark.parametrize('limit',[0,-1,True,1.5])
def test_invalid_diagnostic_budget_rejected(limit):
    with pytest.raises(ValueError):install_return_diagnostics(NS(),lambda row:None,limit=limit)


def execution_fixture():
    planner,p,demo,_,checks,_=fixture();calls=[];sentinel=object()
    demo.planner=NS(native=planner)
    def execute(demo,bridge_mod,real_exec,label,pose,q_path,gripper_pos,args,*,use_attach=False):
        calls.append((label,gripper_pos,args,use_attach))
        return sentinel
    p._jimu_execute_pose_path_stage_base=execute
    rows=install_release_execution_observer(p,lambda row:None)
    return planner,p,demo,checks,calls,sentinel,rows


def test_release_observer_sees_partial_gripper_and_checks_entire_fused_path_without_mutation():
    planner,p,demo,checks,calls,sentinel,rows=execution_fixture()
    demo.robot.get_qpos=lambda:np.array([.719])
    args=NS(_episode_place_released=True)
    path=np.array([[0]*7,[.03]*7])
    result=p._jimu_execute_pose_path_stage_base(demo,None,None,'post_place_clearance_return_to_cycle_start',
        None,path,.719,args)
    assert result is sentinel and calls==[('post_place_clearance_return_to_cycle_start',.719,args,False)]
    row=rows[0]
    assert row['state_unchanged'] and row['diagnostic_complete'] and row['diagnostic_only']
    assert not row['execution_guard'] and not row['native_all_valid']
    assert row['path_points']==2 and row['audited_samples']==len(checks)==4
    assert row['max_gripper_model_error_rad']==pytest.approx(.119)
    assert planner.robot_cfg_dict['kinematics']['lock_joints']['finger_joint']==.6
    assert row['first_invalid'][0]['diagnosed_q']==pytest.approx([.01]*7)
    assert row['first_invalid'][0]['geometry_detail_recorded']


def test_release_invalid_geometry_detail_missing_is_incomplete_not_safe():
    _,p,demo,checks,calls,_,rows=execution_fixture()
    del p._jimu_robot_internal_cube_contacts
    p._jimu_execute_pose_path_stage_base(demo,None,None,'post_place_clearance',None,
        [[0]*7,[.03]*7],.719,NS())
    assert len(checks)==4 and len(calls)==1
    assert not rows[0]['diagnostic_complete'] and not rows[0]['native_all_valid']


@pytest.mark.parametrize('mode',['real_arg','real_executor','prefetch','other_stage'])
def test_release_observer_never_claims_real_prefetch_or_unrelated_stage_evidence(mode):
    _,p,demo,checks,calls,sentinel,rows=execution_fixture()
    options=NS(execute_real=mode=='real_arg',_planning_prefetch_capture_only=mode=='prefetch')
    result=p._jimu_execute_pose_path_stage_base(demo,None,object() if mode=='real_executor' else None,
        'transport' if mode=='other_stage' else 'post_place_clearance',None,[[0]*7,[.03]*7],.719,options)
    assert result is sentinel and len(calls)==1 and rows==checks==[]


def test_release_observer_missing_binding_is_incomplete_not_clear_space():
    _,p,demo,_,calls,sentinel,rows=execution_fixture()
    demo.planner=object()
    assert p._jimu_execute_pose_path_stage_base(demo,None,None,'post_place_clearance',None,
        [[0]*7,[.03]*7],.719,NS()) is sentinel
    assert not rows[0]['diagnostic_complete'] and len(calls)==1


def test_released_model_sync_precedes_observation_without_altering_gripper_command(monkeypatch):
    from rm75_app.workcell import pickplace_gripper_state
    planner,p,demo,_,calls,sentinel,_=execution_fixture();updates=[]
    demo.robot.get_qpos=lambda:np.array([.719])
    def sync(native,actual_demo,args):
        assert native is planner and actual_demo is demo and args._episode_place_released
        updates.append(True)
        planner.robot_cfg_dict['kinematics']['lock_joints']['finger_joint']=.719
    monkeypatch.setattr(pickplace_gripper_state,'update_from_demo',sync)
    rows=install_release_execution_observer(p,lambda row:None,synchronize=True)
    options=NS(_episode_place_released=True)
    assert p._jimu_execute_pose_path_stage_base(demo,None,None,'post_place_clearance',None,
        [[0]*7,[.03]*7],.719,options) is sentinel
    assert updates==[True] and len(calls)==1 and calls[0][1]==.719
    assert rows[0]['max_gripper_model_error_rad']==0 and rows[0]['state_unchanged']
    assert rows[0]['gripper_locks_before_sync']=={'finger_joint':.6}


@pytest.mark.parametrize('missing',['binding','released_state'])
def test_release_model_sync_fails_closed_without_native_binding_or_release(missing):
    from rm75_app.workcell.pickplace_curobo_only import CuroboOnlyUnsupported
    _,p,demo,_,calls,_,_=execution_fixture()
    if missing=='binding':demo.planner=object()
    install_release_execution_observer(p,lambda row:None,synchronize=True)
    with pytest.raises(CuroboOnlyUnsupported):
        p._jimu_execute_pose_path_stage_base(demo,None,None,'post_place_clearance',None,
            [[0]*7,[.03]*7],.719,NS(_episode_place_released=missing!='released_state'))
    assert calls==[]
