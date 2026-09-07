import copy
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rm75_app.workcell.pickplace_lift_diagnostics import install_lift_diagnostics, _returned_rows, nominal_payload_base
from rm75_app.workcell.world_only_contact import WorldOnlyContactUnsupported


def fixture():
    cfg = NS(link_spheres=np.array([[0., 0., 0., .02], [0., 0., .1, .02]]), fixed_transforms=np.eye(4))
    rollout = NS(kinematics=NS(kinematics_config=cfg),
                 primitive_collision_constraint=NS(enabled=True),
                 robot_self_collision_constraint=NS(enabled=True))
    owner = NS(get_all_rollout_instances=lambda: [rollout])
    raw = NS(solution=np.array([[np.zeros(7), np.ones(7)]]),
             success=np.array([[False, False]]), position_error=np.array([[.01, .02]]),
             rotation_error=np.array([[.03, .04]]))
    result = NS(success=False, status='IK_FAIL', raw_result=raw)
    planner = NS(attached_object_active=True, motion_gen=owner, ik_solver=owner,
        robot_cfg_dict={'kinematics': {'lock_joints': {'finger': .6}}},
        get_attached_sphere_count=lambda: 1,
        _world=NS(objects=[NS(name='caller', pose=[0, 0, 0, 1, 0, 0, 0], dims=[.1]*3)]),
        _disabled_collision_links=set(), _disabled_world_obstacles=set())
    checks, queries = [], []
    def check(q):
        checks.append(np.array(q).copy())
        return False, 'SELF_COLLISION'
    planner.check_start_state = check
    planner._compute_world_link_spheres = lambda q: cfg.link_spheres.copy()
    planner._collision_sphere_link_names = lambda: ['attached_object', 'base_link']
    planner._self_collision_buffers = lambda: {}
    planner._self_collision_ignore_pairs = lambda: set()
    planner._extract_pose_components = lambda pose: ([0., 0., .08], [1., 0., 0., 0.])
    planner.diagnose_start_state_self_collision = lambda q, top_k: {'link_pairs': []}
    planner.fk = lambda q: {'position': [0, 0, 0], 'quaternion': [1, 0, 0, 0]}
    direct = NS(_CUROBO_GPU_LOCK=threading.RLock(),
                _current_source_object_name=lambda args: args.source)
    def query(planner, start, goal, **kwargs):
        queries.append((start, goal, kwargs))
        return result
    def segment(planner, demo, args, start, pose_start, pose_goal, *, label, **kwargs):
        old = planner._world
        planner._world = NS(objects=[NS(name='at_query', pose=[0,0,0,1,0,0,0], dims=[.1]*3)])
        try:
            return direct._profile_solve_ik(planner, start, pose_goal, num_seeds=32)
        finally:
            planner._world = old
    direct._profile_solve_ik = query
    direct._plan_short_linear_segment_via_goal_ik = segment
    return direct, planner, result, checks, queries


def run(direct, planner, *, source='gluestick', label='joint_start_tcp_up_lift_center_straight_ik', real=False):
    return direct._plan_short_linear_segment_via_goal_ik(planner, NS(),
        NS(source=source, execute_real=real), np.zeros(7), [0]*7, [1]*7,
        label=label, use_attach=False)


def test_captures_original_attached_query_world_and_preserves_result_seeds_model():
    direct, planner, result, checks, queries = fixture()
    rows = install_lift_diagnostics(direct, lambda row: None)
    before = copy.deepcopy(result.raw_result.__dict__)
    assert run(direct, planner) is result
    assert len(queries) == 1 and queries[0][2] == {'num_seeds': 32}
    assert len(checks) == 3 and len(rows) == 1
    row = rows[0]
    assert row['state_unchanged'] and row['diagnostic_complete'] and row['attached']
    assert row['world_objects'][0]['name'] == 'at_query'
    assert planner._world.objects[0].name == 'caller'
    assert row['start']['box_contacts'][0]['link'] == 'attached_object'
    assert row['goal_pose']['position'] == [0., 0., .08]
    assert row['nominal_goal_payload_base']['contacts'][0]['overlap_m'] == pytest.approx(.02)
    for key, value in before.items():
        np.testing.assert_equal(getattr(result.raw_result, key), value)
    assert [item['position_error_m'] for item in row['returned_rows']] == [.01, .02]
    assert [item['configuration']['joints'][0] for item in row['returned_rows']] == [0, 1]


@pytest.mark.parametrize('kind', ['success', 'detached', 'real', 'unrelated', 'unscoped'])
def test_no_observation_outside_failed_sim_loaded_lift(kind):
    direct, planner, result, checks, queries = fixture()
    rows = install_lift_diagnostics(direct, lambda row: None)
    result.success = kind == 'success'
    planner.attached_object_active = kind != 'detached'
    if kind == 'unscoped':
        actual = direct._profile_solve_ik(planner, [0]*7, [1]*7)
    else:
        actual = run(direct, planner, real=kind == 'real',
                     label='post_place_clearance' if kind == 'unrelated' else 'joint_start_lift_center')
    assert actual is result and rows == checks == [] and len(queries) == 1


def test_limits_only_observations_per_source_not_original_solver_calls():
    direct, planner, result, checks, queries = fixture()
    rows = install_lift_diagnostics(direct, lambda row: None)
    for name in ['gluestick', 'gluestick', 'hongshupian', 'hongshupian']:
        assert run(direct, planner, source=name) is result
    assert len(rows) == 2 and len(queries) == 4 and len(checks) == 6


def test_bad_raw_shape_stays_incomplete_without_rewriting_failure():
    direct, planner, result, _, _ = fixture()
    result.raw_result.position_error = np.array([.01])
    rows = install_lift_diagnostics(direct, lambda row: None)
    assert run(direct, planner) is result
    assert not rows[0]['diagnostic_complete'] and rows[0]['state_unchanged']
    assert 'shape mismatch' in rows[0]['diagnostic_error']


def test_nonfinite_returned_rows_not_silently_dropped_or_serialized_as_nan():
    _, _, result, _, _ = fixture()
    result.raw_result.solution[0, 1, 0] = np.nan
    rows = _returned_rows(result.raw_result)
    assert len(rows) == 2 and rows[0]['finite'] and not rows[1]['finite']
    assert rows[1]['joints'] is None and rows[1]['position_error_m'] is None


def test_collision_state_mutation_fails_closed():
    direct, planner, _, _, _ = fixture()
    def fk(q):
        planner.motion_gen.get_all_rollout_instances()[0].robot_self_collision_constraint.enabled = False
        return {}
    planner.fk = fk
    rows = install_lift_diagnostics(direct, lambda row: None)
    with pytest.raises(WorldOnlyContactUnsupported, match='not preserved'):
        run(direct, planner)
    assert not rows[0]['state_unchanged']


def test_original_exception_propagates_and_context_resets():
    direct, planner, result, checks, _ = fixture()
    calls = []
    def query(*args, **kwargs):
        calls.append(1)
        if len(calls) == 1:
            raise RuntimeError('native failure')
        return result
    direct._profile_solve_ik = query
    rows = install_lift_diagnostics(direct, lambda row: None)
    with pytest.raises(RuntimeError, match='native failure'):
        run(direct, planner)
    assert direct._profile_solve_ik(planner, [0]*7, [1]*7) is result
    assert rows == checks == []


def test_nominal_rigid_payload_uses_target_not_failed_seed_and_native_buffers():
    _, planner, _, _, _ = fixture()
    planner._self_collision_buffers = lambda: {'base_link': .005}
    goal = {'position': [0, 0, .08], 'quaternion': [1, 0, 0, 0]}
    detail = nominal_payload_base(planner, np.zeros(7), goal, np.ones(7))
    assert detail['base_comparison_max_delta_m'] == 0
    assert detail['contacts'][0]['overlap_m'] == pytest.approx(.025)
    assert not detail['contacts'][0]['pair_ignored']
    assert not detail['physical_geometry_qualified']
    goal['position'] = [0, 0, .3]
    assert nominal_payload_base(planner, np.zeros(7), goal, np.ones(7))['contacts'] == []


def test_nominal_comparison_rejects_moving_base_geometry():
    _, planner, _, _, _ = fixture()
    original = planner._compute_world_link_spheres
    def spheres(q):
        result = original(q)
        result[1, 0] += q[0]
        return result
    planner._compute_world_link_spheres = spheres
    with pytest.raises(ValueError, match='base spheres changed'):
        nominal_payload_base(planner, np.zeros(7),
                             {'position': [0,0,.08], 'quaternion': [1,0,0,0]}, np.ones(7))


@pytest.mark.parametrize('limit', [True, 0, -1, 1.5])
def test_invalid_limit_rejected(limit):
    with pytest.raises(ValueError):
        install_lift_diagnostics(NS(), lambda row: None, per_source_limit=limit)
