from types import SimpleNamespace
import threading

import pytest

from rm75_app.workcell.pickplace_curobo_only import serialize_return_planning, preserve_planner_world


@pytest.mark.parametrize('fail', [False, True])
def test_return_mask_lifetime_is_locked_including_restore_and_nested_plan(fail):
    lock = threading.RLock()
    masks = set()
    checks = []

    def contender():
        acquired = lock.acquire(blocking=False)
        checks.append(acquired)
        if acquired:
            lock.release()

    def joint_plan():
        thread = threading.Thread(target=contender)
        thread.start()
        thread.join(timeout=2)
        assert not thread.is_alive()
        assert masks == {'finger'}
        if fail:
            raise ValueError('solver failure')
        return 'original result'

    def prelift():
        masks.add('finger')
        try:
            return direct._plan_return_to_start_joint_curobo()
        finally:
            masks.clear()
            thread = threading.Thread(target=contender)
            thread.start()
            thread.join(timeout=2)
            assert not thread.is_alive()

    direct = SimpleNamespace(_CUROBO_GPU_LOCK=lock,
        _plan_return_to_start_joint_curobo=joint_plan,
        _plan_return_to_start_prelift_rescue_curobo=prelift)
    serialize_return_planning(direct)
    if fail:
        with pytest.raises(ValueError, match='solver failure'):
            direct._plan_return_to_start_prelift_rescue_curobo()
    else:
        assert direct._plan_return_to_start_prelift_rescue_curobo() == 'original result'
    assert checks == [False, False]
    assert not masks
    thread = threading.Thread(target=contender)
    thread.start()
    thread.join(timeout=2)
    assert checks[-1] is True


@pytest.mark.parametrize('fail', [False, True])
def test_world_restore_includes_table_cached_rows_all_owners_and_signature(fail):
    updates = []
    p = SimpleNamespace(_world=SimpleNamespace(objects=[SimpleNamespace(name='table')]),
        _disabled_world_obstacles={'active_target_object'}, _persistent_world_signature=('table',),
        motion_gen=SimpleNamespace(update_world=lambda w: updates.append(('motion', w))),
        ik_solver=SimpleNamespace(update_world=lambda w: updates.append(('ik', w))),
        _update_cuda_graph_batch_ik_world=lambda w: updates.append(('cached_ik', w)),
        world_collision_checker_obstacle_names=lambda: ['table', 'active_target_object', 'worker_only'])
    def setter(names, *, enabled):
        if enabled: p._disabled_world_obstacles.difference_update(names)
        else: p._disabled_world_obstacles.update(names)
    p.set_world_obstacles_enabled = setter
    def transaction():
        with preserve_planner_world(p):
            p._world = SimpleNamespace(objects=[SimpleNamespace(name='worker_only')])
            p._disabled_world_obstacles = {'table'}
            p._persistent_world_signature = ('worker_only',)
            p._last_world_changed = True
            if fail: raise ValueError('native solver failed')
    if fail:
        with pytest.raises(ValueError): transaction()
    else: transaction()
    assert p._persistent_world_signature == ('table',)
    assert not hasattr(p, '_last_world_changed')
    assert p._disabled_world_obstacles == {'active_target_object', 'worker_only'}
    assert [name for name, _ in updates] == ['motion', 'ik', 'cached_ik']
    assert all(w.objects[0].name == 'table' for _, w in updates)


@pytest.mark.parametrize('failure', [None, ValueError, KeyboardInterrupt, 'evidence_sink'])
def test_nested_phase_participant_restores_on_success_exception_and_base_exception(failure):
    from rm75_app.workcell.phase_state import register
    state={'excluded_source':'right_roof_triangle'};rows=[]
    p=SimpleNamespace(_world=SimpleNamespace(objects=[SimpleNamespace(name='table')]),
        _disabled_world_obstacles=set(), motion_gen=SimpleNamespace(update_world=lambda w:None),
        ik_solver=SimpleNamespace(update_world=lambda w:None),
        _update_cuda_graph_batch_ik_world=lambda w:None,
        world_collision_checker_obstacle_names=lambda:['table','stale_source'])
    def setter(names,*,enabled):
        if enabled:p._disabled_world_obstacles.difference_update(names)
        else:p._disabled_world_obstacles.update(names)
    p.set_world_obstacles_enabled=setter
    def restore(saved):state.clear();state.update(saved)
    def emit(row):
        if failure=='evidence_sink' and row['phase']=='temporary_world_exit':
            raise KeyboardInterrupt('sink failed')
        rows.append(row)
    register(p,'contact',SimpleNamespace(capture=lambda:state.copy(),restore=restore,emit=emit))
    def run():
        with preserve_planner_world(p,label='prelift'):
            state['excluded_source']='temporary'
            with preserve_planner_world(p,label='return'):
                state['excluded_source']=None
                p._disabled_world_obstacles.add('table')
                if failure and failure!='evidence_sink':raise failure('original failure')
            assert state=={'excluded_source':'temporary'}
    if failure:
        with pytest.raises(KeyboardInterrupt if failure=='evidence_sink' else failure):run()
    else:run()
    assert state=={'excluded_source':'right_roof_triangle'}
    assert p._disabled_world_obstacles=={'stale_source'}
    assert all(row['adapter_state_restored'] for row in rows if row['phase']=='restored')


def test_participants_are_planner_local_and_do_not_enable_required_disabled_object():
    from rm75_app.workcell.phase_state import register, evidence
    p=SimpleNamespace(_world=SimpleNamespace(objects=[SimpleNamespace(name='table'),SimpleNamespace(name='neighbor')]),
        _disabled_world_obstacles={'table','neighbor','stale_source'})
    other=SimpleNamespace()
    register(p,'contact',SimpleNamespace(capture=lambda:{'excluded_source':'roof'},restore=lambda s:None,emit=lambda r:None))
    assert not hasattr(other,'_rm75_phase_participants')
    row=evidence(p)
    assert row['required_objects_disabled']==['neighbor','table']
    assert row['absent_cache_rows_disabled']==['stale_source']
    assert p._disabled_world_obstacles=={'table','neighbor','stale_source'}
