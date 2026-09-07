from types import SimpleNamespace as NS
from concurrent.futures import ThreadPoolExecutor
import pytest
import torch
from rm75_app.workcell.world_only_contact import world_only_fingers, WorldOnlyContactUnsupported, FINGER_LINKS


def make_planner():
    names = {'arm': 0, 'gripper_Left_1_Link': 1, 'gripper_Right_1_Link': 2, 'gripper_base_link': 3}
    config = NS(link_name_to_idx_map=names, link_sphere_idx_map=torch.tensor([0, 1, 2, 3]))
    world = NS(enabled=True, forward=lambda spheres, **kw: spheres.clone())
    constraint = NS(enabled=True, forward=lambda spheres, **kw: spheres.clone())
    self_check = NS(enabled=True, forward=lambda spheres: spheres.clone())
    rollout = NS(kinematics=NS(kinematics_config=config), primitive_collision_cost=world,
                 primitive_collision_constraint=constraint, robot_self_collision_constraint=self_check)
    owner = NS(use_cuda_graph=False, get_all_rollout_instances=lambda: [rollout])
    return NS(motion_gen=owner, ik_solver=owner, _disabled_collision_links=set(),
              _disabled_world_obstacles=set(), _cuda_graph_batch_ik_solvers={}), rollout


def test_only_finger_world_input_changes_self_and_base_keep_geometry():
    planner, rollout = make_planner()
    spheres = torch.ones((1, 2, 4, 4))
    original = spheres.clone()
    forward = rollout.primitive_collision_cost.forward
    with world_only_fingers(planner) as evidence:
        for cost in (rollout.primitive_collision_cost, rollout.primitive_collision_constraint):
            result = cost.forward(spheres)
            assert torch.all(result[..., [1, 2], 3] == -100)
            assert torch.equal(result[..., [0, 3], :], original[..., [0, 3], :])
        assert torch.equal(rollout.robot_self_collision_constraint.forward(spheres), original)
        assert torch.equal(spheres, original)
        assert evidence['world_filter_calls'] == 2
    assert rollout.primitive_collision_cost.forward is forward
    assert 'gripper_base_link' not in FINGER_LINKS


def test_restore_on_planner_exception():
    planner, rollout = make_planner()
    forward = rollout.primitive_collision_constraint.forward
    with pytest.raises(RuntimeError, match='planning failed'):
        with world_only_fingers(planner):
            raise RuntimeError('planning failed')
    assert rollout.primitive_collision_constraint.forward is forward


@pytest.mark.parametrize('field,value', [
    ('_disabled_collision_links', {'arm'}), ('_disabled_world_obstacles', {'table'}),
    ('_cuda_graph_batch_ik_solvers', {'cached': object()}),
])
def test_rejects_preexisting_unsafe_or_graph_state(field, value):
    planner, _ = make_planner()
    setattr(planner, field, value)
    with pytest.raises(WorldOnlyContactUnsupported):
        with world_only_fingers(planner):
            pytest.fail('unsafe state accepted')


def test_rejects_graph_and_missing_self_checks():
    planner, rollout = make_planner()
    planner.motion_gen.use_cuda_graph = True
    with pytest.raises(WorldOnlyContactUnsupported):
        with world_only_fingers(planner):
            pass
    planner.motion_gen.use_cuda_graph = False
    rollout.robot_self_collision_constraint.enabled = False
    with pytest.raises(WorldOnlyContactUnsupported):
        with world_only_fingers(planner):
            pass


def test_other_thread_does_not_inherit_exception():
    planner, rollout = make_planner()
    spheres = torch.ones((1, 1, 4, 4))
    with world_only_fingers(planner), ThreadPoolExecutor(1) as pool:
        result = pool.submit(rollout.primitive_collision_cost.forward, spheres).result()
        assert torch.equal(result, spheres)


def test_shape_change_fails_closed_and_restores():
    planner, rollout = make_planner()
    forward = rollout.primitive_collision_cost.forward
    with pytest.raises(WorldOnlyContactUnsupported):
        with world_only_fingers(planner):
            rollout.primitive_collision_cost.forward(torch.ones((1, 1, 5, 4)))
    assert rollout.primitive_collision_cost.forward is forward


def native_scope(*, slot=0, real=False, delta_z=-0.07, delta_x=0.):
    import threading
    import numpy as np
    from rm75_app.workcell.contact_audit import install_contact_audit
    planner, rollout = make_planner()
    calls, rows = [], []
    direct = NS(_CUROBO_GPU_LOCK=threading.RLock(),
                _current_source_object_name=lambda args: args.object_name,
                _normalize_disabled_world_collision_links=lambda p, links: list(links),
                run_targeted_place_episode_curobo_direct=lambda *a, **kw: None)
    def matrix(pose):
        value = np.eye(4)
        value[:3, 3] = pose
        return value
    direct._pose_to_matrix_from_pose_obj = matrix
    direct._set_world_collision_for_links = lambda *a, **kw: pytest.fail('shared sphere toggle called')
    direct._refresh_curobo_world = lambda *a, **kw: calls.append(kw)
    def segment(*a, **kw):
        spheres = torch.ones((1, 1, 4, 4))
        result = rollout.primitive_collision_constraint.forward(spheres)
        assert result[0, 0, 1, 3] == -100
        assert result[0, 0, 3, 3] == 1
        assert torch.equal(rollout.robot_self_collision_constraint.forward(spheres), spheres)
        return ['path']
    direct._plan_constrained_linear_segment = segment
    def final(p, demo, args, choice, lookup):
        direct._refresh_curobo_world(p, demo, args, label='candidate_final_approach', include_table=False)
        label = 'candidate_final_approach_gripper_world_relaxed'
        links = direct._set_world_collision_for_links(p, ['gripper_Left_1_Link', 'gripper_base_link'], enabled=False, label=label)
        try:
            return direct._plan_constrained_linear_segment(p, demo, args, None,
                [0, 0, 0.1], [delta_x, 0, 0.1 + delta_z], label=label)
        finally:
            direct._set_world_collision_for_links(p, links, enabled=True, label=label)
    direct._apply_deferred_two_step_final_approach = final
    install_contact_audit(direct, rows.append, tray_final_descent=True)
    demo = NS(scene_capture_cache_ref={'objects': {'right_wall': {'jimu_tray_slot_index': slot}}})
    args = NS(object_name='right_wall', execute_real=real, curobo_table_collision=True)
    return direct, planner, demo, args, calls, rows


def test_qualified_descent_uses_world_copy_and_retains_table_target():
    direct, planner, demo, args, calls, rows = native_scope()
    assert direct._apply_deferred_two_step_final_approach(planner, demo, args, {'label': 'candidate'}, {}) == ['path']
    assert calls[0]['include_table'] is True and calls[0]['include_active_object'] is True
    assert any(r['event'] == 'tray_world_only_filter_end' and r['world_filter_calls'] == 1 for r in rows)


@pytest.mark.parametrize('changes', [{'slot': None}, {'real': True}, {'delta_z': 0.02}, {'delta_z': -0.2}, {'delta_x': 0.01}])
def test_rejects_nontray_real_upward_long_or_lateral_motion(changes):
    from rm75_app.workcell.contact_audit import StrictContactNotSupported
    direct, planner, demo, args, _, rows = native_scope(**changes)
    with pytest.raises(StrictContactNotSupported):
        direct._apply_deferred_two_step_final_approach(planner, demo, args, {'label': 'candidate'}, {})
    assert not any(r['event'] == 'tray_world_only_filter_enter' for r in rows)


def test_earlier_paired_ik_is_not_authorized():
    from rm75_app.workcell.contact_audit import StrictContactNotSupported
    direct, planner, _, _, _, _ = native_scope()
    with pytest.raises(StrictContactNotSupported):
        direct._set_world_collision_for_links(planner, ['gripper_Left_Support_Link'], enabled=False,
                                             label='winner_chain_jimu_parallel_grasp_place_paired_relation_ik')
