import copy
import threading
from types import SimpleNamespace as NS
import pytest
import torch
from rm75_app.workcell.contact_audit import StrictContactNotSupported
from rm75_app.workcell.transport_contact import payload_contact_config, is_transport, install_transport_contact


def test_jimu_read_only_details_use_all_links_and_preserve_native_values():
    import numpy as np
    from rm75_app.workcell.transport_contact import jimu_collision_details
    calls=[];planner=object();q=np.arange(7)
    def geometry(p,joints,disabled_world_collision_links=None):
        assert p is planner and disabled_world_collision_links is None
        calls.append(joints.copy());return [{'robot_link':'left_pad','clearance_m':-.004}]
    portable=NS(_jimu_robot_world_obstacle_contacts=geometry,_jimu_robot_internal_cube_contacts=geometry,
                _jimu_curobo_raw_world_collision_snapshot=lambda p,joints:{'constraint':[1.0],'nonzero_count':1})
    detail=jimu_collision_details(portable,planner,q)
    assert detail['geometry_detail_recorded'] and len(calls)==2
    assert detail['robot_world_obstacle_contacts'][0]['clearance_m']==-.004
    assert detail['curobo_raw_world_collision']['constraint']==[1.0]
    np.testing.assert_array_equal(q,np.arange(7))


def test_missing_geometry_diagnostic_cannot_claim_complete_evidence():
    from rm75_app.workcell.transport_contact import jimu_collision_details
    assert not jimu_collision_details(NS(),object(),[0]*7)['geometry_detail_recorded']


@pytest.mark.parametrize('valid', [False, True])
def test_jimu_diagnostic_retains_current_world_payload_and_collision_result(valid):
    from rm75_app.workcell.transport_contact import install_read_only_jimu_diagnostics
    planner, direct, _, rows, _, _, close, _ = native_fixture()
    calls=[]
    def check(q):
        calls.append(q.tolist())
        return valid, 'VALID' if valid else 'WORLD_COLLISION'
    planner.check_start_state=check
    def forbidden(*a, **kw):
        raise AssertionError('Diagnostic must not perform ablations or change collision state')
    planner.clear_world=forbidden
    planner.set_world_obstacles_enabled=forbidden
    direct._set_world_collision_for_links=forbidden
    portable=NS(direct=direct,_jimu_start_collision_diagnosis=forbidden)
    world=planner._world
    try:
        install_read_only_jimu_diagnostics(portable,rows.append)
        result=portable._jimu_start_collision_diagnosis(planner,NS(),
            [{'start_q':[0]*7}], 'joint_transport_hover', ['attached_object','left_pad'])
        assert result['valid'] is valid and result['valid_after_removing']==[]
        assert result['world_obstacle_names']==['virtual_table_plane']
        assert result['attached_sphere_summary']=={'active':True,'count':6}
        assert planner._world is world and planner.attached_object_active
        assert not planner._disabled_collision_links and len(calls)==1
        assert rows[-1]['event']=='jimu_read_only_collision_diagnostic'
    finally:close()


def test_jimu_read_only_diagnostic_does_not_suppress_native_failure():
    from rm75_app.workcell.transport_contact import install_read_only_jimu_diagnostics
    def fail(q):raise RuntimeError('native check failure')
    portable=NS(direct=NS(_CUROBO_GPU_LOCK=threading.RLock()),_jimu_start_collision_diagnosis=lambda *a:None)
    install_read_only_jimu_diagnostics(portable,lambda row:None)
    with pytest.raises(RuntimeError,match='native check failure'):
        portable._jimu_start_collision_diagnosis(NS(check_start_state=fail),NS(),[{'start_q':[0]*7}],'transport',[])


def test_payload_pairs_are_local_and_do_not_change_geometry_or_robot_adjacency():
    raw = {'robot_cfg': {'kinematics': {'collision_link_names': ['base_link', 'link_1', 'attached_object', 'left_pad'],
        'collision_spheres': 'unchanged.yml', 'self_collision_ignore': {
            'base_link': ['link_1', 'attached_object'], 'attached_object': ['base_link'], 'left_pad': []}}}}
    before = copy.deepcopy(raw)
    result = payload_contact_config(raw)['robot_cfg']['kinematics']
    assert result['self_collision_ignore'] == {'base_link': ['link_1'], 'left_pad': ['attached_object']}
    assert result['collision_spheres'] == 'unchanged.yml' and raw == before


@pytest.mark.parametrize('label,wanted', [
    ('joint_transport_hover_pairs_after_lift_seq_01', True), ('transport_to_hover', True),
    ('transport_hover_final_contact', False), ('transport_hover_release_precheck', False),
    ('winner_chain_paired_relation_ik', False), ('joint_start_tcp_up_lift_world_z', False),
])
def test_phase_classification(label, wanted):
    assert is_transport(label) is wanted


def native_fixture():
    rows, calls, checked = [], [], []
    config = NS(link_sphere_idx_map=torch.tensor([0, 1]), link_name_to_idx_map={'base_link': 0, 'left_pad': 1})
    world = NS(enabled=True, forward=lambda spheres: spheres.clone())
    rollout = NS(kinematics=NS(kinematics_config=config), primitive_collision_constraint=world,
                 robot_self_collision_constraint=NS(enabled=True))
    owner = NS(use_cuda_graph=False, get_all_rollout_instances=lambda: [rollout])
    class Planner:
        _load_robot_cfg_dict = lambda self: {'kinematics': {'collision_link_names': ['attached_object', 'left_pad']}}
        _build_motion_gen = lambda self: None
        solve_batch_start_goal_ik = lambda self, *a, **kw: kw
    planner = Planner()
    planner.motion_gen = owner
    planner.ik_solver = owner
    planner._disabled_collision_links = set()
    planner._disabled_world_obstacles = {'active_target_object'}
    planner._cuda_graph_batch_ik_solvers = {}
    planner._world = NS(objects=[NS(name='virtual_table_plane')])
    planner.attached_object_active = True
    planner.collision_enabled = True
    planner.config = NS(self_collision_check=True)
    planner.get_attached_sphere_count = lambda: 6
    def check(q):
        checked.append(q)
        return q != 'collision', 'WORLD_COLLISION' if q == 'collision' else None
    planner.check_start_state = check
    direct = NS(_CUROBO_GPU_LOCK=threading.RLock(),
                _normalize_disabled_world_collision_links=lambda p, links: list(links),
                _refresh_curobo_world=lambda *a, **kw: calls.append(kw),
                _current_source_object_name=lambda args: 'right_wall',
                _build_virtual_table_cuboid=lambda args: {'name': 'virtual_table_plane'})
    direct._single_obstacle_start_collision_relief=lambda planner,*args,**kwargs:'unrelated_holder'
    def evaluate(*a, **kw):
        calls.append(kw)
        return [{'q_path': ['start', 'middle', 'end']}]
    for name in ('_evaluate_curobo_pose_candidates', '_evaluate_curobo_pose_candidates_goalset', '_evaluate_curobo_pose_candidates_multi_start'):
        setattr(direct, name, evaluate)
    close = install_transport_contact(direct, Planner, rows.append)
    args = NS(execute_real=False, curobo_table_collision=True)
    return planner, direct, args, rows, calls, checked, close, world


def test_original_relief_cannot_remove_an_unrelated_holder_from_the_lift_or_transport_world():
    planner,direct,_,rows,_,_,close,_=native_fixture()
    try:
        assert direct._single_obstacle_start_collision_relief(planner,[0]*7,already_excluded={'right_wall'}) is None
        assert rows[-1]['event']=='contact_obstacle_relief_refused'
        assert rows[-1]['obstacle']=='unrelated_holder'
        assert planner._disabled_world_obstacles=={'active_target_object'}
    finally:close()


def test_transport_checks_every_sample_with_table_and_no_exemption():
    planner, direct, args, rows, calls, checked, close, _ = native_fixture()
    try:
        assert direct._transport_attached_contact_disabled_links(planner, ['left_pad']) == []
        result = direct._evaluate_curobo_pose_candidates_multi_start(planner, None, args, [],
            label='joint_transport_hover', disabled_world_collision_links=['left_pad'], exclude_object_names={'right_wall'})
        assert checked == ['start', 'middle', 'end']
        assert calls[0]['disabled_world_collision_links'] == [] and calls[0]['include_table'] is True
        assert rows[-1]['samples'] == 3 and len(result) == 1
        assert planner.solve_batch_start_goal_ik(use_cuda_graph_batch=True)['use_cuda_graph_batch'] is False
    finally:
        close()


def test_container_exclusion_is_restored_for_loaded_lift_and_transport():
    planner,direct,args,rows,calls,checked,close,_=native_fixture()
    try:
        direct._refresh_curobo_world(planner,None,args,label='joint_start_tcp_up_lift_world',
            exclude_object_names={'right_wall','unrelated_holder'})
        assert calls[-1]['exclude_object_names']=={'right_wall'} and calls[-1]['include_table'] is True
        direct._evaluate_curobo_pose_candidates_multi_start(planner,None,args,[],label='transport_to_hover',
            exclude_object_names={'right_wall','unrelated_holder'})
        assert calls[-2]['exclude_object_names']=={'right_wall'}
        assert any(row.get('restored_objects')==['unrelated_holder'] for row in rows)
        assert checked==['start','middle','end']
    finally:close()


@pytest.mark.parametrize('label',['winner_chain_ik_preselect_grasp','winner_chain_ik_preselect_grasp_contact'])
def test_ik_screen_keeps_table_before_selecting_transport_goals(label):
    planner,direct,args,rows,calls,_,close,_=native_fixture()
    try:
        direct._refresh_curobo_world(planner,None,args,label=label,include_table=False,include_active_object=False)
        assert calls[-1]['include_table'] is True
        assert calls[-1]['include_active_object'] is False
        assert rows[-1]['event']=='ik_preselect_table_restored'
        assert rows[-1]['seeds_or_candidates_changed'] is False
    finally:close()


@pytest.mark.parametrize('problem', ['payload', 'self', 'world', 'table', 'real', 'other_obstacle'])
def test_transport_prerequisites_fail_closed(problem):
    planner, direct, args, _, _, _, close, _ = native_fixture()
    if problem == 'payload': planner.attached_object_active = False
    if problem == 'self': planner.config.self_collision_check = False
    if problem == 'world': planner.collision_enabled = False
    if problem == 'table': planner._world.objects = []
    if problem == 'real': args.execute_real = True
    if problem == 'other_obstacle': planner._disabled_world_obstacles.add('neighbor')
    try:
        with pytest.raises(StrictContactNotSupported):
            direct._evaluate_curobo_pose_candidates_multi_start(planner, None, args, [], label='transport_to_hover')
    finally:
        close()


def test_contact_stage_world_copy_restores_and_cannot_leak_into_transport():
    planner, direct, args, rows, _, _, close, world = native_fixture()
    original = world.forward
    try:
        direct._set_world_collision_for_links(planner, ['left_pad'], enabled=False, label='grasp_final_approach')
        result = world.forward(torch.ones((1, 1, 2, 4)))
        assert result[0, 0, 1, 3] == -100 and result[0, 0, 0, 3] == 1
        assert not planner._disabled_collision_links
        with pytest.raises(StrictContactNotSupported):
            direct._evaluate_curobo_pose_candidates_multi_start(planner, None, args, [], label='transport_to_hover')
        direct._set_world_collision_for_links(planner, ['left_pad'], enabled=True, label='grasp_final_approach')
        assert world.forward is original
        with pytest.raises(StrictContactNotSupported):
            direct._set_world_collision_for_links(planner, ['left_pad'], enabled=False, label='joint_transport_hover')
    finally:
        close()
    assert world.forward is original


@pytest.mark.parametrize('raises', [False, True])
def test_native_start_check_restores_flags_on_early_return_and_exception(raises):
    from rm75_app.workcell.transport_contact import preserve_start_check_flags
    class Cost:
        enabled = True
        def enable_cost(self): self.enabled = True
        def disable_cost(self): self.enabled = False
    world, self_cost = Cost(), Cost()
    def native_check(q):
        self_cost.disable_cost()
        if raises:
            raise RuntimeError('native diagnostic failure')
        return False, 'WORLD_COLLISION'
    mg = NS(check_start_state=native_check, rollout_fn=NS(
        primitive_collision_constraint=world, robot_self_collision_constraint=self_cost))
    preserve_start_check_flags(mg)
    if raises:
        with pytest.raises(RuntimeError, match='native diagnostic failure'):
            mg.check_start_state([])
    else:
        assert mg.check_start_state([]) == (False, 'WORLD_COLLISION')
    assert world.enabled and self_cost.enabled


def test_transport_collision_sample_is_not_accepted():
    planner, direct, args, _, _, checked, close, _ = native_fixture()
    def check(q):
        checked.append(q)
        return q != 'middle', 'WORLD_COLLISION'
    planner.check_start_state = check
    try:
        with pytest.raises(StrictContactNotSupported) as exc:
            direct._evaluate_curobo_pose_candidates_multi_start(planner, None, args, [], label='transport_to_hover')
        assert exc.value.evidence['reason'] == 'transport_sample_collision:WORLD_COLLISION'
        assert checked == ['start', 'middle']
    finally:
        close()


def test_omitted_table_metadata_is_not_permission_to_disable_present_table():
    planner, direct, _, _, _, _, close, _ = native_fixture()
    planner._disabled_world_obstacles.add('virtual_table_plane')
    try:
        with pytest.raises(StrictContactNotSupported):
            direct._set_world_collision_for_links(planner, ['left_pad'], enabled=False, label='paired_relation_ik')
        planner._world.objects = []
        direct._set_world_collision_for_links(planner, ['left_pad'], enabled=False, label='paired_relation_ik')
        direct._set_world_collision_for_links(planner, ['left_pad'], enabled=True, label='paired_relation_ik')
    finally:
        close()


@pytest.mark.parametrize('problem', [None, 'no_identity', 'not_excluded', 'no_payload', 'zero_spheres', 'present', 'neighbor'])
def test_attached_source_cache_requires_identity_and_live_payload(problem):
    planner, direct, args, _, _, _, close, _ = native_fixture()
    planner._disabled_world_obstacles.add('scene_obstacle_right_wall')
    try:
        if problem != 'no_identity':
            direct._refresh_curobo_world(planner, None, args, label='lift_world',
                exclude_object_names=set() if problem == 'not_excluded' else {'right_wall'})
        if problem == 'no_payload': planner.attached_object_active = False
        if problem == 'zero_spheres': planner.get_attached_sphere_count = lambda: 0
        if problem == 'present': planner._world.objects.append(NS(name='scene_obstacle_right_wall'))
        if problem == 'neighbor': planner._disabled_world_obstacles.add('scene_obstacle_neighbor')
        if problem:
            with pytest.raises(StrictContactNotSupported):
                direct._set_world_collision_for_links(planner, ['left_pad'], enabled=False, label='first_lift')
        else:
            direct._set_world_collision_for_links(planner, ['left_pad'], enabled=False, label='first_lift')
            direct._set_world_collision_for_links(planner, ['left_pad'], enabled=True, label='first_lift')
    finally:
        close()
