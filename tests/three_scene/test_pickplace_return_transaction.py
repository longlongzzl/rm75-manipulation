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
