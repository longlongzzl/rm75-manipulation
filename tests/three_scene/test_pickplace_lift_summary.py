import json
from tools.summarize_native_gaps import lift_ik_summary


def test_failed_lift_summary_keeps_failure_denominator_and_excludes_raw_world_pose_joints():
    secret='private-example-data'
    raw={'source':'gluestick','native_success':False,'diagnostic_complete':True,
         'state_unchanged':True,'attached':True,'payload_spheres':6,
         'goal_pose':secret,'world_objects':secret,'start':{'joints':secret,'valid':True},
         'returned_rows':[{'index':0,'finite':True,'native_success':False,'position_error_m':.00001,
             'configuration':{'joints':secret,'fk':secret,'native_status':'SELF_COLLISION',
                 'self_collision':{'link_pairs':[{'link_a':'attached_object','link_b':'base_link',
                     'overlap':.008,'count':1,'joints':secret}]}}},
             {'index':1,'finite':False,'native_success':False,'joints':secret}],
         'nominal_goal_payload_base':{'contacts':[{'link_a':'attached_object','link_b':'base_link',
             'overlap_m':.008,'pair_ignored':False,'spheres':secret}], 'physical_geometry_qualified':False}}
    row=lift_ik_summary(raw)
    assert secret not in json.dumps(row)
    assert not row['native_success'] and row['start']['valid']
    assert len(row['returned_rows'])==2 and not row['returned_rows'][1]['finite']
    assert row['returned_rows'][0]['configuration']['self_link_pairs'][0]['overlap']==.008
    assert row['nominal_goal_payload_base']['contacts'][0]['overlap_m']==.008


def test_pre_nominal_run_missing_evidence_not_invented():
    row=lift_ik_summary({'diagnostic_complete':False,'native_success':False})
    assert row['nominal_goal_payload_base'] is None
    assert row['returned_rows']==[] and row['state_unchanged'] is None
