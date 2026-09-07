import threading
from types import SimpleNamespace as NS
import pytest
from rm75_app.workcell.transport_contact import guard_jimu_near_ik


@pytest.mark.parametrize('valid',[False,True])
def test_near_ik_pose_only_promotion_requires_actual_collision_validity(valid):
    rows=[];result=NS(success=False,status='IK_FAIL',goal_joint=None,debug={'position_error':1.e-6})
    def original_accept(result,*a,**kw):
        result.success=True;result.status='SuccessNearIK';result.goal_joint=[0]*7
        result.debug['jimu_near_ik_fallback']=True
        return result
    portable=NS(_jimu_maybe_accept_near_ik_result=original_accept,
        _ORIGINAL_CUROBO_SOLVE_IK=object())
    class Planner:
        collision_enabled=True;config=NS(self_collision_check=True);_disabled_collision_links=set()
        attached_object_active=False;_disabled_world_obstacles=set();_world=NS(objects=[])
        def solve_ik(self):return portable._jimu_maybe_accept_near_ik_result(result,[0]*7,NS())
        solve_batch_start_goal_ik=solve_ik;_solve_batch_start_goal_ik_cuda_graph_once=solve_ik
        def check_start_state(self,q):
            assert q==[0]*7
            return valid,'VALID' if valid else 'WORLD_COLLISION'
    portable.direct=NS(curobo_wrapper=NS(RM75CuRoboPlanner=Planner),_CUROBO_GPU_LOCK=threading.RLock())
    portable._install_jimu_near_ik_fallback=lambda *a:None
    guard_jimu_near_ik(portable,rows.append)
    portable._install_jimu_near_ik_fallback(NS())
    output=Planner().solve_ik()
    assert output.success is valid and rows[-1]['accepted'] is valid
    assert result.success is False and result.status=='IK_FAIL' and result.goal_joint is None
    assert result.debug=={'position_error':1.e-6}
    assert valid or output is result


def test_missing_solver_context_never_promotes_failed_ik():
    def promote(result,*args):result.success=True;return result
    portable=NS(_jimu_maybe_accept_near_ik_result=promote,_install_jimu_near_ik_fallback=lambda:None)
    guard_jimu_near_ik(portable,lambda row:None)
    result=NS(success=False,debug={})
    assert portable._jimu_maybe_accept_near_ik_result(result) is result and not result.success


@pytest.mark.parametrize('fail',[False,True])
def test_grasp_contact_scope_is_consumed_once_and_restored(monkeypatch,fail):
    from contextlib import contextmanager
    from rm75_app.workcell import transport_contact as contact
    state={'filtered':False};seen=[];rows=[]
    @contextmanager
    def scope(planner,links,*,allowed_disabled_objects):
        assert links==contact.FINGER_LINKS
        assert allowed_disabled_objects=={'active_target_object'}
        state['filtered']=True
        try:yield {'links':sorted(links)}
        finally:state['filtered']=False
    monkeypatch.setattr(contact,'world_only_links',scope)
    planner=object();args=NS(execute_real=False)
    def native(args,p,*pos,**kwargs):
        seen.append(state['filtered'])
        if state['filtered'] and fail:raise RuntimeError('original IK error')
        return pos,kwargs
    direct=NS(_refresh_curobo_world=lambda *a,**kw:None,
        _profile_fast_chain_solve_batch_start_goal_ik=native,_CUROBO_GPU_LOCK=threading.RLock())
    contact.install_jimu_grasp_ik_contact(direct,rows.append)
    for label in ('winner_chain_ik_preselect_grasp','joint_transport_hover'):
        direct._refresh_curobo_world(planner,None,args,label=label)
        direct._profile_fast_chain_solve_batch_start_goal_ik(args,planner,1,num_seeds=32)
    direct._refresh_curobo_world(planner,None,args,label='winner_chain_ik_preselect_grasp_contact')
    if fail:
        with pytest.raises(RuntimeError,match='original IK error'):
            direct._profile_fast_chain_solve_batch_start_goal_ik(args,planner,1,num_seeds=32)
    else:
        assert direct._profile_fast_chain_solve_batch_start_goal_ik(args,planner,1,num_seeds=32)==((1,),{'num_seeds':32})
    assert state['filtered'] is False
    direct._profile_fast_chain_solve_batch_start_goal_ik(args,planner,1,num_seeds=32)
    assert seen==[False,False,True,False]
    assert [r['event'] for r in rows]==['grasp_contact_ik_filter_enter','grasp_contact_ik_filter_exit']


def test_grasp_ik_scope_rejects_real_mode():
    from rm75_app.workcell.transport_contact import install_jimu_grasp_ik_contact,WorldOnlyContactUnsupported
    direct=NS(_refresh_curobo_world=lambda *a,**kw:None,
        _profile_fast_chain_solve_batch_start_goal_ik=lambda *a,**kw:pytest.fail('must not solve'))
    planner=object();args=NS(execute_real=True)
    install_jimu_grasp_ik_contact(direct,lambda row:None)
    direct._refresh_curobo_world(planner,None,args,label='winner_chain_ik_preselect_grasp_contact')
    with pytest.raises(WorldOnlyContactUnsupported,match='simulation_only'):
        direct._profile_fast_chain_solve_batch_start_goal_ik(args,planner)
