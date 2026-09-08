import ast
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rm75_app.workcell import jimu_roof_ik_diagnostics as diagnostic
from rm75_app.workcell.pickplace_curobo_only import CuroboOnlyUnsupported


def result(solution,errors):
    solution=np.asarray(solution,dtype=np.float32)
    shape=solution.shape[:-1]
    return NS(success=False,status='IK_FAIL',debug={'position_error':float(np.asarray(errors).reshape(-1)[0])},
        raw_result=NS(solution=solution,success=np.zeros(shape,dtype=bool),
                      position_error=np.asarray(errors).reshape(shape),rotation_error=np.zeros(shape)))


def test_nearest_raw_seed_and_its_error_match_exact_native_extraction_not_first_debug_scalar():
    source=Path(__file__).resolve().parents[2]/'rm75_app/_vendor/working_snapshot/Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py'
    node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef)
              and n.name=='_jimu_extract_near_ik_solution')
    scope={'np':np,'_jimu_to_numpy':np.asarray}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),scope)
    item=result([[np.full(7,2.),np.full(7,.1)]],[[1e-5,.2]])
    rows,nearest=diagnostic.result_rows(item,0,1,np.zeros(7))
    assert nearest==1 and rows[nearest]['position_error_m']==.2 and item.debug['position_error']==1e-5
    expected=scope['_jimu_extract_near_ik_solution'](item.raw_result,np.zeros(7))
    np.testing.assert_array_equal(rows[nearest]['joints'],expected)


def test_batch_goal_identity_and_seed_errors_remain_aligned():
    item=result([[np.full(7,.1)],[np.full(7,.2)]],[[.01],[.02]])
    item.debug['batch_index']=1
    rows,nearest=diagnostic.result_rows(item,1,2,np.zeros(7))
    assert nearest==0 and rows[0]['position_error_m']==.02
    assert rows[0]['joints'][0]==pytest.approx(.2)


def test_scalar_error_is_not_broadcast_across_seeds():
    item=result([[np.zeros(7),np.ones(7)]],[[.01,.02]])
    item.raw_result.position_error=np.array([.01])
    with pytest.raises(ValueError,match='count mismatch'):
        diagnostic.result_rows(item,0,1,np.zeros(7))


def fixture(monkeypatch, **audit_options):
    planner=NS(scope=False,world='old',_extract_pose_components=lambda p:(p[:3],p[3:]))
    item=result([[np.full(7,.1),np.full(7,.2)]],[[.01,.02]])
    items=[item];calls=[];observed=[]
    def query(options,p,starts,goals,*,num_seeds):
        calls.append((starts,goals,num_seeds,p.scope));return items
    def refresh(p,demo,args,**kwargs):p.world=kwargs['label'];return 'refreshed'
    def toggle(p,links,*,enabled,label):p.scope=not enabled;return []
    direct=NS(_CUROBO_GPU_LOCK=threading.RLock(),_current_source_object_name=lambda args:args.object_name,
              _refresh_curobo_world=refresh,_set_world_collision_for_links=toggle,
              _profile_fast_chain_solve_batch_start_goal_ik=query)
    monkeypatch.setattr(diagnostic,'_state',lambda p:{'scope':p.scope,'world':p.world})
    def details(p,q):
        observed.append(p.scope)
        return {'fk':{'position':[0.,0.,0.]},'valid':False,'native_status':'WORLD_COLLISION'}
    monkeypatch.setattr(diagnostic,'_configuration_evidence',details)
    monkeypatch.setattr(diagnostic,'nominal_gripper_base_boxes',lambda *args:{'goals':[]})
    rows=diagnostic.install_roof_ik_diagnostics(direct,lambda row:None,**audit_options)
    options=NS(object_name='right_roof_triangle',execute_real=False)
    starts=[np.zeros(7)];goals=[[0,0,.1,1,0,0,0]]
    return planner,direct,options,starts,goals,items,calls,rows,observed


def test_observation_is_inside_query_contact_scope_and_preserves_original_arguments_results(monkeypatch):
    p,d,options,starts,goals,items,calls,rows,observed=fixture(monkeypatch)
    assert d._refresh_curobo_world(p,None,options,label='winner_chain_ik_preselect_grasp_contact')=='refreshed'
    d._set_world_collision_for_links(p,['finger'],enabled=False,label='winner_paired_relation_ik')
    actual=d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32)
    d._set_world_collision_for_links(p,[],enabled=True,label='winner_paired_relation_ik')
    assert actual is items and len(calls)==1 and calls[0]==(starts,goals,32,True)
    assert observed==[True] and rows[0]['state_unchanged'] and rows[0]['diagnostic_complete']
    assert rows[0]['phase']=='paired_place' and rows[0]['goals'][0]['goal_pose']['position']==[0,0,.1]


def test_limits_diagnostics_per_source_and_phase_without_dropping_native_calls(monkeypatch):
    p,d,options,starts,goals,_,calls,rows,_=fixture(monkeypatch)
    for source in ('right_roof_triangle','left_roof_triangle'):
        options.object_name=source
        for phase in ('pregrasp','grasp_contact'):
            d._refresh_curobo_world(p,None,options,label='winner_chain_ik_preselect_'+phase)
            for _ in range(2):d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32)
    assert len(calls)==8 and len(rows)==4


@pytest.mark.parametrize('kind',['not_roof','real','unknown_phase'])
def test_only_qualified_original_roof_sim_phases_are_observed(monkeypatch,kind):
    p,d,options,starts,goals,items,calls,rows,observed=fixture(monkeypatch)
    options.object_name='right_wall' if kind=='not_roof' else 'right_roof_triangle'
    options.execute_real=kind=='real'
    d._refresh_curobo_world(p,None,options,label='other' if kind=='unknown_phase' else 'pregrasp')
    assert d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32) is items
    assert rows==observed==[] and len(calls)==1


def test_prefetch_and_foreground_have_separate_evidence_budgets(monkeypatch):
    p,d,options,starts,goals,_,calls,rows,_=fixture(monkeypatch)
    d._refresh_curobo_world(p,None,options,label='winner_chain_ik_preselect_grasp')
    for prefetch in (True,True,False):
        options._planning_prefetch_capture_only=prefetch
        d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32)
    assert len(calls)==3 and [row['prefetch'] for row in rows]==[True,False]


@pytest.mark.parametrize('kind',['complete','empty','missing_phase','incomplete','changed_state'])
def test_requested_roof_audit_needs_observed_complete_scopes(kind):
    rows=[dict(source='front_roof_triangle',phase=phase,prefetch=True,
        diagnostic_complete=True,state_unchanged=True) for phase in ('pregrasp','grasp','paired_place')]
    if kind=='empty':rows=[]
    elif kind=='missing_phase':rows.pop()
    elif kind=='incomplete':rows[0]['diagnostic_complete']=False
    elif kind=='changed_state':rows[0]['state_unchanged']=False
    report=diagnostic.roof_audit_status(rows,['front_roof_triangle'])
    assert report['passed'] is (kind=='complete')
    assert report['prefetch_batches']==len(rows) and report['foreground_batches']==0


def test_missing_raw_mapping_stays_incomplete_and_keeps_native_failure(monkeypatch):
    p,d,options,starts,goals,items,_,rows,_=fixture(monkeypatch)
    items[0].raw_result.position_error=np.array([.01])
    d._refresh_curobo_world(p,None,options,label='pregrasp')
    assert d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32) is items
    assert not rows[0]['diagnostic_complete'] and rows[0]['state_unchanged']


def test_unexpected_collision_state_mutation_fails_closed(monkeypatch):
    p,d,options,starts,goals,_,_,rows,_=fixture(monkeypatch)
    def change(p,q):p.scope=True;return {'fk':{'position':[0,0,0]}}
    monkeypatch.setattr(diagnostic,'_configuration_evidence',change)
    d._refresh_curobo_world(p,None,options,label='pregrasp')
    with pytest.raises(CuroboOnlyUnsupported,match='changed collision state'):
        d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32)
    assert not rows[0]['state_unchanged']


def test_exact_original_grasp_refresh_label_precedes_pregrasp_batch(monkeypatch):
    p,d,options,starts,goals,_,_,rows,_=fixture(monkeypatch)
    d._refresh_curobo_world(p,None,options,label='winner_chain_ik_preselect_grasp')
    d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32)
    assert rows[0]['phase']=='pregrasp'


def test_explicit_tennis_scope_reuses_original_observer_without_observing_roofs(monkeypatch):
    p,d,options,starts,goals,items,calls,rows,_=fixture(monkeypatch,
        sources={'tennis'},event_name='pickplace_level_ik_batch_diagnostic')
    d._refresh_curobo_world(p,None,options,label='pregrasp')
    assert d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32) is items
    assert not rows
    options.object_name='tennis'
    assert d._profile_fast_chain_solve_batch_start_goal_ik(options,p,starts,goals,num_seeds=32) is items
    assert len(calls)==2 and rows[0]['event']=='pickplace_level_ik_batch_diagnostic'
    assert rows[0]['state_unchanged'] and rows[0]['diagnostic_complete']


def test_reviewed_native_paired_query_has_hover_then_release_goal_order():
    path=Path(__file__).resolve().parents[2]/'rm75_app/_vendor/working_snapshot/pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef)
              and n.name=='_fast_chain_evaluate_paired_relation_records')
    calls=[n for n in ast.walk(node) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)
           and n.func.id=='_profile_fast_chain_solve_batch_start_goal_ik']
    assert len(calls)==1
    assert ast.unparse(calls[0].args[2])=='q_grasps + q_grasps'
    assert ast.unparse(calls[0].args[3])=='hover_poses + release_poses'


def geometry_planner():
    def fk(q):return dict(position=[q[0],0,0],quaternion=[np.cos(q[0]/2),0,0,np.sin(q[0]/2)])
    def spheres(q):
        pose=fk(q);center=np.array([.1,0,0])@diagnostic.quaternion_matrix(pose['quaternion']).T+pose['position']
        return np.array([[*center,.02],[100,100,100,.02]])
    objects=[NS(name='wall',dims=[.1,.1,.1],pose=[1.1,0,0,1,0,0,0]),
             NS(name='mesh',dims=None)]
    return NS(_collision_sphere_link_names=lambda:['gripper_base_link','other'],
        _compute_world_link_spheres=spheres,fk=fk,_disabled_collision_links=set(),
        _disabled_world_obstacles=set(),_world=NS(objects=objects))


def test_nominal_gripper_base_uses_original_rigid_offset_and_keeps_radii_world():
    p=geometry_planner()
    goals=[dict(position=[1,0,0],quaternion=[1,0,0,0]),
           dict(position=[2,0,0],quaternion=[1,0,0,0])]
    row=diagnostic.nominal_gripper_base_boxes(p,[0],[.5],goals)
    assert row['sphere_count']==1 and row['rigid_comparison_max_delta_m']<1e-12
    assert [g['enabled_box_overlap'] for g in row['goals']]==[True,False]
    assert row['goals'][0]['box_contacts'][0]['overlap_m']==pytest.approx(.07)
    assert row['non_box_objects_not_analytically_audited']==['mesh']
    assert not row['physical_geometry_qualified'] and p._disabled_world_obstacles==set()


@pytest.mark.parametrize('kind',['disabled_link','non_rigid','missing_sphere'])
def test_unqualified_nominal_geometry_rejected(kind):
    p=geometry_planner()
    if kind=='disabled_link':p._disabled_collision_links.add('gripper_base_link')
    elif kind=='missing_sphere':p._collision_sphere_link_names=lambda:['other','other']
    else:
        original=p._compute_world_link_spheres
        def wrong(q):
            value=original(q);value[0,0]+=q[0]*.01;return value
        p._compute_world_link_spheres=wrong
    with pytest.raises(ValueError):
        diagnostic.nominal_gripper_base_boxes(p,[0],[.5],[])
