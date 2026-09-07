from types import SimpleNamespace as NS
import threading
import numpy as np
import pytest

from rm75_app.workcell.pickplace_curobo_only import CuroboOnlyUnsupported
from rm75_app.workcell.pickplace_release_contact import audit_release_path,release_target,install_release_contact


def fixture(problem=None):
    target=NS(name='scene_obstacle_shuazi',dims=[.1,.1,.1],pose=[0,0,0,1,0,0,0])
    p=NS(collision_enabled=True,config=NS(self_collision_check=True),attached_object_active=False,
        _disabled_collision_links=set(),_disabled_world_obstacles={'active_target_object'},_world=NS(objects=[target]))
    checks=[]
    def setter(names,*,enabled):
        if enabled:p._disabled_world_obstacles.difference_update(names)
        else:p._disabled_world_obstacles.update(names)
        return names
    p.set_world_obstacles_enabled=setter
    def spheres(q):
        x=.056+float(q[0])
        if problem=='growing':x=.056-float(q[0])
        if problem=='nan':x=float('nan')
        return np.array([[x,0,0,.01]])
    p._compute_world_link_spheres=spheres
    p._collision_sphere_link_names=lambda:['gripper_base_link' if problem=='palm' else 'left_pad']
    def check(q):
        checks.append(q.copy())
        assert p._disabled_world_obstacles=={'active_target_object',target.name}
        if problem=='native_error':raise ValueError('native error')
        return problem not in ('neighbor','self'),'WORLD_OR_SELF_COLLISION'
    p.check_start_state=check
    if problem=='masked':p._disabled_collision_links={'left_pad'}
    if problem=='missing_neighbor':p._disabled_world_obstacles.add('neighbor')
    return p,target,checks


def test_finger_target_only_contact_checks_dense_path_and_restores_world():
    p,target,checks=fixture();result=audit_release_path(p,[[0],[.02]],target)
    assert result['initial_penetration_m']==pytest.approx(.004)
    assert result['final_penetration_m']==0
    assert result['audited_samples']==len(checks)==3
    assert result['world_exempt_links']==[]
    assert p._disabled_world_obstacles=={'active_target_object'}


@pytest.mark.parametrize('problem',['palm','neighbor','self','growing','masked','missing_neighbor','nan'])
def test_no_other_pair_state_or_increased_penetration_is_exempt(problem):
    p,target,checks=fixture(problem);before=p._disabled_world_obstacles.copy()
    with pytest.raises(CuroboOnlyUnsupported):audit_release_path(p,[[0],[.02]],target)
    assert p._disabled_world_obstacles==before


def test_restore_after_native_check_exception():
    p,target,checks=fixture('native_error')
    with pytest.raises(ValueError):audit_release_path(p,[[0],[.02]],target)
    assert p._disabled_world_obstacles=={'active_target_object'}


@pytest.mark.parametrize('problem',['not_released','real','wrong_source','mesh'])
def test_target_identity_and_release_qualification_are_mandatory(problem):
    p,target,_=fixture();args=NS(execute_real=False,_episode_place_released=True)
    if problem=='not_released':args._episode_place_released=False
    if problem=='real':args.execute_real=True
    if problem=='mesh':target.file_path='not-a-cuboid.obj'
    with pytest.raises(CuroboOnlyUnsupported):
        release_target(p,args,'other' if problem=='wrong_source' else 'shuazi')


def test_rejected_segment_restores_target_and_preserves_next_native_candidate():
    p,target,_=fixture('growing');native_check=p.check_start_state
    p.check_start_state=lambda q:native_check(q) if target.name in p._disabled_world_obstacles else (False,'target_contact')
    tried=[]
    def segment(planner,demo,args,start_q,pose_start,pose_goal,**kwargs):
        assert target.name in planner._disabled_world_obstacles
        tried.append(pose_goal)
        return [[0],[pose_goal]]
    direct=NS(_plan_constrained_linear_segment=segment,_CUROBO_GPU_LOCK=threading.RLock(),
              _current_source_object_name=lambda args:'shuazi')
    install_release_contact(direct)
    args=NS(execute_real=False,_episode_place_released=True)
    paths=[direct._plan_constrained_linear_segment(p,None,args,[0],None,goal,
            label='post_place_clearance_candidate') for goal in (.02,-.02)]
    assert paths[0] is None and paths[1] is not None
    assert tried==[.02,-.02] and p._disabled_world_obstacles=={'active_target_object'}


def test_native_error_is_not_swallowed_as_release_candidate_rejection():
    p,target,_=fixture();native_check=p.check_start_state
    p.check_start_state=lambda q:native_check(q) if target.name in p._disabled_world_obstacles else (False,'target_contact')
    def segment(*args,**kwargs):raise RuntimeError('native GPU error')
    direct=NS(_plan_constrained_linear_segment=segment,_CUROBO_GPU_LOCK=threading.RLock(),
              _current_source_object_name=lambda args:'shuazi')
    install_release_contact(direct)
    with pytest.raises(RuntimeError,match='native GPU error'):
        direct._plan_constrained_linear_segment(p,None,NS(execute_real=False,_episode_place_released=True),
            [0],None,.02,label='post_place_clearance_candidate')
    assert p._disabled_world_obstacles=={'active_target_object'}


@pytest.mark.parametrize('pose',[[float('nan'),0,0,1,0,0,0],[0,0,0,0,0,0,0]])
def test_invalid_target_pose_cannot_disappear_from_analytic_collision_check(pose):
    p,target,checks=fixture();target.pose=pose
    args=NS(execute_real=False,_episode_place_released=True)
    with pytest.raises(CuroboOnlyUnsupported):release_target(p,args,'shuazi')
    with pytest.raises(CuroboOnlyUnsupported):audit_release_path(p,[[0],[.02]],target)
    assert not checks and p._disabled_world_obstacles=={'active_target_object'}


def selection_fixture(problem):
    p,target,_=fixture(problem);native_check=p.check_start_state
    p.check_start_state=lambda q:native_check(q) if target.name in p._disabled_world_obstacles else (False,'target_contact')
    original_result={'success':True,'status':'NATIVE_SELECTIVE_CLEAR'}
    direct=NS(_plan_constrained_linear_segment=lambda *a,**kw:None,
        _CUROBO_GPU_LOCK=threading.RLock(),_current_source_object_name=lambda args:'shuazi',
        _audit_post_place_clearance_selective_contact_path=lambda *a,**kw:original_result)
    install_release_contact(direct)
    return p,target,direct,original_result


def test_valid_reverse_endpoints_do_not_hide_intermediate_contact_growth():
    p,target,direct,_=selection_fixture('growing')
    args=NS(execute_real=False,_episode_place_released=True)
    path=[[0],[.02],[0]]
    assert not direct._rm75_clearance_reverse_path_reusable(p,None,args,path)
    assert path==[[0],[.02],[0]] and p._disabled_world_obstacles=={'active_target_object'}
    assert direct._clearance_selection_audits[-1]['kind']=='reverse_reuse'
    assert direct._rm75_clearance_reverse_path_reusable(p,None,args,[[0],[-.02]])


def test_selective_candidate_rejection_preserves_original_result_and_next_candidate():
    p,target,direct,original=selection_fixture('growing')
    args=NS(execute_real=False,_episode_place_released=True)
    results=[direct._audit_post_place_clearance_selective_contact_path(p,None,args,[[0],[end]])
             for end in (.02,-.02)]
    assert not results[0]['success'] and results[1]['success']
    assert original=={'success':True,'status':'NATIVE_SELECTIVE_CLEAR'}
    assert p._disabled_world_obstacles=={'active_target_object'}


def test_reverse_query_error_is_not_converted_to_retry_or_success():
    p,target,direct,_=selection_fixture('native_error')
    with pytest.raises(ValueError,match='native error'):
        direct._rm75_clearance_reverse_path_reusable(p,None,NS(execute_real=False,_episode_place_released=True),[[0],[.02]])
    assert p._disabled_world_obstacles=={'active_target_object'}
