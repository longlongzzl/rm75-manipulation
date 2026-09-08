import threading
from types import SimpleNamespace as NS

import pytest

from rm75_app.workcell import pickplace_focused_diagnostics as focused
from rm75_app.workcell.pickplace_curobo_only import CuroboOnlyUnsupported


@pytest.mark.parametrize('accepted', [True, False])
def test_existing_reverse_is_observed_without_selection_or_new_solver(monkeypatch, accepted):
    direct = NS(_CUROBO_GPU_LOCK=threading.RLock(), _current_source_object_name=lambda args: 'tennis')
    monkeypatch.setattr(focused, '_state', lambda p: {'unchanged': True})
    monkeypatch.setattr(focused, 'validate_clearance_path', lambda *a: (_ for _ in ()).throw(CuroboOnlyUnsupported('contact')))
    monkeypatch.setattr(focused, 'release_target', lambda *a: object())
    def audit(*a):
        if not accepted:
            raise CuroboOnlyUnsupported('contact deepens')
        return {'all_valid': True, 'audited_samples': 4}
    monkeypatch.setattr(focused, 'audit_release_path', audit)
    path = [[0], [1]]; rows = []
    row = focused.observe_reverse(direct, object(), NS(execute_real=False), path, rows.append)
    assert row['accepted_by_same_full_path_gate'] is accepted
    assert row['selected'] is False and row['new_solver_calls'] == 0
    assert row['state_unchanged'] and path == [[0], [1]] and rows == [row]


def test_diagnostic_state_mutation_fails_closed(monkeypatch):
    direct = NS(_CUROBO_GPU_LOCK=threading.RLock(), _current_source_object_name=lambda args: 'tennis')
    states = iter([{'version': 1}, {'version': 2}])
    monkeypatch.setattr(focused, '_state', lambda p: next(states))
    monkeypatch.setattr(focused, 'validate_clearance_path', lambda *a: {'all_valid': True})
    rows = []
    with pytest.raises(CuroboOnlyUnsupported, match='changed collision state'):
        focused.observe_reverse(direct, object(), NS(execute_real=False), [[0], [1]], rows.append)
    assert rows[0]['state_unchanged'] is False


@pytest.mark.parametrize('real,prefetch', [(False, False), (True, False), (False, True)])
def test_native_endpoint_result_and_cleanup_are_preserved(monkeypatch, real, prefetch):
    result = {'valid': False, 'status': 'native rejection'}
    class Planner:
        def diagnose_start_state_world_collision(self, *args):
            return result
    original = Planner.diagnose_start_state_world_collision
    seen = []
    monkeypatch.setattr(focused, 'install_lift_diagnostics', lambda *a: None)
    monkeypatch.setattr(focused, 'observe_reverse', lambda *a: seen.append(a))
    direct = NS(_current_source_object_name=lambda args: 'tennis')
    cleanup = focused.install(direct, Planner, lambda r: None, requested_source='tennis')
    def run_targeted_place_episode_curobo_direct():
        args = NS(execute_real=real, _planning_prefetch_capture_only=prefetch)
        reverse_clearance_path = [[0], [1]]
        return Planner().diagnose_start_state_world_collision(reverse_clearance_path[-1])
    assert run_targeted_place_episode_curobo_direct() is result
    assert run_targeted_place_episode_curobo_direct() is result
    assert len(seen) == int(not real and not prefetch)
    cleanup()
    assert Planner.diagnose_start_state_world_collision is original


def test_other_source_cannot_enter_focused_observer():
    with pytest.raises(ValueError, match='three reviewed sources'):
        focused.install(None, None, None, requested_source='bi')


def test_grasp_diagnostic_preserves_one_original_batch_and_scope(monkeypatch):
    from rm75_app.workcell import jimu_roof_ik_diagnostics as roof
    from rm75_app.workcell import pickplace_lift_diagnostics as lift
    original_result = [NS(success=False), NS(success=False)]
    calls=[]; rows=[]
    def query(options, planner, starts, goals, *, num_seeds):
        calls.append((starts,goals,num_seeds));return original_result
    direct=NS(_CUROBO_GPU_LOCK=threading.RLock(),_current_source_object_name=lambda args:args.source,
        _refresh_curobo_world=lambda *a,**k:'original-refresh',
        _profile_fast_chain_solve_batch_start_goal_ik=query)
    monkeypatch.setattr(focused,'_state',lambda p:{'original_scope':True})
    monkeypatch.setattr(roof,'result_rows',lambda result,index,count,start:(
        [{'finite':True,'native_success':False,'joints':[index]*7}],0))
    monkeypatch.setattr(lift,'_configuration_evidence',lambda p,q:{'valid':False,'joints':q})
    focused.install_grasp_diagnostic(direct,rows.append)
    planner=object(); options=NS(source='gluestick',execute_real=False)
    starts=[[0]*7,[1]*7];goals=[object(),object()]
    assert direct._refresh_curobo_world(planner,None,options,label='winner_chain_ik_preselect_grasp_contact')=='original-refresh'
    for _ in range(2):
        assert direct._profile_fast_chain_solve_batch_start_goal_ik(options,planner,starts,goals,num_seeds=32) is original_result
    assert len(calls)==2 and all(row==(starts,goals,32) for row in calls)
    assert len(rows)==1 and rows[0]['goal_count']==2
    assert rows[0]['diagnostic_complete'] and rows[0]['state_unchanged'] and rows[0]['returned_rows_unchanged']
