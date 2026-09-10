from types import SimpleNamespace as NS
import pytest

from rm75_app.workcell.failed_object_ik_search import require_search, merge_results, install


@pytest.mark.parametrize('mode', ['real','preview'])
def test_search_refuses_non_simulation(mode):
    with pytest.raises(ValueError):
        require_search(dict(task='pickplace',mode=mode),{'pickplace':{
            'fixed_scene_format':'native_world','simulation_contact_policy':'transport_world_checked_compatibility'}})


def test_search_refuses_unchecked_collision_policy():
    with pytest.raises(ValueError):
        require_search(dict(task='pickplace',mode='sim'),{'pickplace':{'fixed_scene_format':'native_world'}})


def test_retry_never_overwrites_existing_success_or_accepts_colliding_endpoint():
    old = [NS(success=True),NS(success=False),NS(success=False),NS(success=False)]
    new = [NS(success=True),NS(success=True),NS(success=True),NS(success=False)]
    merged = merge_results(old,new,[True,True,False,True])
    assert merged == [old[0],new[1],old[2],old[3]]


def test_retry_refuses_misaligned_result_counts():
    with pytest.raises(ValueError,match='identity'):
        merge_results([NS(success=False)],[],[])


def test_retry_requires_explicit_bounded_budget():
    with pytest.raises(ValueError):
        install(None,None,num_seeds=4096)


def test_single_goal_retry_keeps_its_own_seed_errors_inside_larger_batch():
    import numpy as np
    from rm75_app.workcell.jimu_roof_ik_diagnostics import result_rows
    raw=NS(solution=np.array([[[0.]*7,[1.]*7]]),success=np.array([[False,True]]),
           position_error=np.array([[.02,.0001]]),rotation_error=np.array([[.2,.001]]))
    result=NS(raw_result=raw,debug={'ik_search_goal_count':1,'original_goal_index':19})
    rows,_=result_rows(result,19,26,np.zeros(7))
    assert len(rows)==2
    assert rows[0]['native_success'] is False
    assert rows[1]['native_success'] is True
    assert rows[1]['position_error_m']==pytest.approx(.0001)
    assert rows[1]['joints']==[1.]*7


def test_retry_freezes_buffers_before_next_solver_call():
    import numpy as np
    from rm75_app.workcell.failed_object_ik_search import freeze_retry_result
    raw=NS(solution=np.ones((1,2,7)),success=np.ones((1,2),dtype=bool),
           position_error=np.full((1,2),.001),rotation_error=np.full((1,2),.002))
    result=NS(raw_result=raw,goal_joint=np.ones(7),debug={})
    frozen=freeze_retry_result(result)
    raw.solution[:]=0;raw.success[:]=False;raw.position_error[:]=10;raw.rotation_error[:]=20
    result.goal_joint[:]=0
    assert frozen.raw_result.solution.sum()==14
    assert frozen.raw_result.success.all()
    assert frozen.raw_result.position_error[0,1]==.001
    assert frozen.raw_result.rotation_error[0,1]==.002
    assert frozen.goal_joint.sum()==7


def test_installed_retry_is_one_goal_at_a_time_and_preserves_each_seed(monkeypatch):
    import threading
    import numpy as np
    import rm75_app.workcell.failed_object_ik_search as search
    calls=[];events=[]
    starts=[np.zeros(7) for _ in range(3)]
    goals=[([float(i),0.,0.],[1.,0.,0.,0.]) for i in range(3)]
    baseline_raw=NS(solution=np.zeros((3,1,7)),success=np.array([[True],[False],[False]]),
                    position_error=np.zeros((3,1)),rotation_error=np.zeros((3,1)))
    original=[NS(success=i==0,raw_result=baseline_raw,goal_joint=np.zeros(7) if i==0 else None,
                 debug={'batch_index':i}) for i in range(3)]
    reused=NS(solution=np.zeros((1,1,7)),success=np.ones((1,1),dtype=bool),
              position_error=np.zeros((1,1)),rotation_error=np.zeros((1,1)))
    def batch(options,planner,qs,poses,*,num_seeds):
        calls.append((len(poses),num_seeds))
        if num_seeds==32:return original
        assert len(qs)==len(poses)==1
        index=poses[0][0][0]
        reused.solution[:]=index
        reused.position_error[:]=index*.0001
        return [NS(success=True,raw_result=reused,goal_joint=reused.solution[0,0],debug={})]
    direct=NS(_CUROBO_GPU_LOCK=threading.RLock(),
        _profile_fast_chain_solve_batch_start_goal_ik=batch,
        _profile_solve_ik=lambda *a,**kw:None,
        _plan_short_linear_segment_via_goal_ik=lambda *a,**kw:None,
        _current_source_object_name=lambda options:'gluestick')
    monkeypatch.setattr(search,'_state',lambda planner:{'geometry':'unchanged'})
    monkeypatch.setattr(search,'_configuration_evidence',lambda planner,q:{'valid':True,'joints':q.tolist()})
    planner=NS(_extract_pose_components=lambda pose:pose)
    close=search.install(direct,events.append,num_seeds=128)
    try:
        result=direct._profile_fast_chain_solve_batch_start_goal_ik(
            NS(execute_real=False),planner,starts,goals,num_seeds=32)
        assert calls==[(3,32),(1,128),(1,128)]
        assert result[0] is original[0]
        assert result[1].goal_joint.tolist()==[1.]*7
        assert result[2].goal_joint.tolist()==[2.]*7
        assert events[0]['rows'][1]['raw_rows'][0]['position_error_m']==pytest.approx(.0001)
        assert events[0]['newly_accepted']==2
        assert events[0]['state_unchanged'] and events[0]['raw_results_unchanged']
    finally:close()
    assert direct._profile_fast_chain_solve_batch_start_goal_ik is batch
