import json
from copy import deepcopy
import pytest
from tools.summarize_native_gaps import event_summary, roof_ik_summary


def evidence():
    secret='private-raw-world-joints'
    return dict(event='jimu_roof_ik_batch_diagnostic',source='right_roof_triangle',phase='paired_place',
        goal_count=1,native_success_count=0,diagnostic_complete=True,state_unchanged=True,
        original_near_position_threshold=1e-4,original_near_rotation_threshold=1e-3,
        world_objects=secret,goals=[dict(goal_index=0,native_success=False,native_status='IK_FAIL',
            goal_pose=secret,legacy_nearest_seed_index=1,debug_position_error_m=1e-5,
            debug_rotation_error_native=1e-5,seeds=[
                dict(seed_index=0,native_success=False,finite=True,position_error_m=1e-5,
                    rotation_error_native=1e-5,joints=secret),
                dict(seed_index=1,native_success=False,finite=True,position_error_m=.2,
                    rotation_error_native=.1,joints=secret)])],
        configuration_details=[dict(goal_index=0,seed_index=1,joints=secret,fk=secret,valid=False,
            box_contacts=[dict(link='link_5',obstacle='table',overlap_m=.01,geometry=secret)],
            self_collision={'link_pairs':[dict(link_a='link_5',link_b='link_1',overlap=.01,q=secret)]})])


def test_mismatch_uses_nearest_seed_and_bounded_export_not_debug_first_row():
    row=roof_ik_summary(evidence())
    assert row['debug_nearest_error_mismatch_count']==1
    assert row['debug_passes_but_nearest_fails_original_thresholds_count']==1
    assert row['goals'][0]['nearest_position_error_m']==.2
    assert row['configuration_details'][0]['box_contacts'][0]['overlap_m']==.01
    assert 'private-raw-world-joints' not in json.dumps(row)


@pytest.mark.parametrize('mode',['one_seed','unavailable','debug_fails'])
def test_no_unproven_threshold_mismatch(mode):
    raw=evidence();goal=raw['goals'][0]
    if mode=='one_seed':
        goal['seeds']=goal['seeds'][:1];goal['legacy_nearest_seed_index']=0
    elif mode=='unavailable':goal['debug_position_error_m']=None
    else:goal['debug_position_error_m']=.3
    row=roof_ik_summary(raw)
    assert row['debug_passes_but_nearest_fails_original_thresholds_count']==0
    if mode=='one_seed':assert row['debug_nearest_error_mismatch_count']==0


@pytest.mark.parametrize('bare',[True,False])
def test_worker_and_standalone_events_keep_roof_evidence(tmp_path,bare):
    path=tmp_path/'events.jsonl';row=evidence()
    path.write_text(json.dumps(row if bare else {'kind':'contact_audit','evidence':row})+'\n')
    result=event_summary(path,bare=bare)
    assert result['event_counts']['jimu_roof_ik_batch_diagnostic']==1
    assert result['roof_ik_diagnostics'][0]['captured_goal_count']==1


def test_incomplete_diagnostic_is_not_promoted_to_observed_success():
    row=roof_ik_summary(dict(goal_count=192,diagnostic_complete=False,
        diagnostic_error='ValueError: private-raw-world-joints',goals=[]))
    assert not row['diagnostic_complete'] and row['captured_goal_count']==0
    assert row['goal_count']==192 and row['diagnostic_error_type']=='ValueError'
    assert 'private-raw-world-joints' not in json.dumps(row)


def test_large_batch_examples_bounded_but_all_goals_counted():
    raw=evidence();goal=raw['goals'][0]
    raw['goals']=[dict(deepcopy(goal),goal_index=i) for i in range(20)]
    raw['goal_count']=20
    row=roof_ik_summary(raw)
    assert row['captured_goal_count']==row['error_comparison_available_count']==20
    assert row['debug_nearest_error_mismatch_count']==20 and len(row['goals'])==8
    assert row['goal_examples_only'] is True


def test_nominal_geometry_keeps_all_goals_and_pairs_not_coordinates():
    raw=evidence()
    raw['nominal_gripper_base_boxes']={'sphere_count':1,'physical_geometry_qualified':False,
        'goals':[{'goal_index':i,'enabled_box_overlap':i<2,'position':'private-coordinate',
            'box_contacts':[{'link':'gripper_base_link','obstacle':'wall','enabled':True,
                             'overlap_m':.01+i*.001}] if i<2 else []} for i in range(3)]}
    row=roof_ik_summary(raw)['nominal_gripper_base_boxes']
    assert row['goal_count']==3 and row['goals_with_enabled_box_overlap']==2
    assert row['contact_pairs'][0]['goal_count']==2
    assert row['contact_pairs'][0]['max_overlap_m']==pytest.approx(.011)
    assert 'private-coordinate' not in json.dumps(row)


def test_hover_release_pairing_keeps_every_candidate_in_denominator():
    raw=evidence();raw['goals']=[dict(deepcopy(raw['goals'][0]),goal_index=i) for i in range(4)]
    raw['nominal_gripper_base_boxes']={'goals':[dict(goal_index=i,
        enabled_box_overlap=i!=0,box_contacts=[]) for i in range(4)]}
    pairs=roof_ik_summary(raw)['nominal_gripper_base_boxes']['paired_candidates']
    assert pairs==dict(count=2,native_goal_order='hover_then_release',
        hover_with_enabled_box_overlap=1,release_with_enabled_box_overlap=2,
        either_goal_with_enabled_box_overlap=2)
