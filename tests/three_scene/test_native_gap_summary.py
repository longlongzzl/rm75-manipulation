import json
from tools.summarize_native_gaps import negative_pairs,event_summary,pusht_summary


def test_collision_summary_excludes_world_geometry_joint_states_and_paths():
    detail={'diagnosed_q':[1]*7,'robot_world_obstacle_contacts':[
        {'robot_link':'link_5','obstacle':'virtual_table_plane','clearance_m':-.025,
         'sphere_center':[1,2,3],'file_path':'/internal/secret'},
        {'robot_link':'left_pad','obstacle':'tray','clearance_m':.002}]}
    assert negative_pairs(detail)==[{'robot_link':'link_5','obstacle':'virtual_table_plane','clearance_m':-.025}]


def test_summary_preserves_rejections_and_does_not_count_empty_transport(tmp_path):
    path=tmp_path/'events.jsonl'
    rows=[{'event':'jimu_near_ik_collision_checked','accepted':False,'diagnostic':{}},
          {'event':'jimu_near_ik_collision_checked','accepted':True},
          {'event':'transport_full_world_audit','samples':0}]
    path.write_text('\n'.join(json.dumps({'kind':'contact_audit','evidence':row}) for row in rows))
    report=event_summary(path)
    assert report['near_ik_promotions']=={'accepted':1,'rejected':1}
    assert report['transport_audits']==[] and report['transport_all_world_links_checked'] is False


def test_pusht_summary_keeps_failure_and_metrics_without_trajectory():
    raw={'gpu_backend':'curobo2','complete_chain':True,'validation_success':False,
         'error':'RuntimeError: internal/path',
         'stages':[{'stage':'push','positions':[[1]*7]*2,'times':[0.,2.],
                    'max_tcp_speed_mps':.01,'max_joint_speed_rad_s':.1,'max_joint_accel_rad_s2':.2}]}
    row=pusht_summary(raw)
    assert row['validation_success'] is False and row['complete_chain'] is True
    assert row['error_type']=='RuntimeError' and row['stages'][0]['duration_s']==2.
    assert 'positions' not in row['stages'][0] and 'times' not in row['stages'][0]


def test_bare_native_contact_events_keep_same_audit_semantics(tmp_path):
    path=tmp_path/'contact.jsonl'
    path.write_text(json.dumps({'event':'transport_full_world_audit','samples':3,
        'world_exempt_links':[],'payload_spheres':6,'step_id':'transport'}))
    result=event_summary(path,bare=True)
    assert result['transport_samples']==3 and result['transport_all_world_links_checked']


def test_pusht_partial_stage_and_injected_gate_evidence_are_not_lost_or_exported_raw():
    raw={'complete_chain':False,'validation_success':False,
         'events':[{'event':'push_stage_audited','stage':'approach','samples':1324,
                    'duration_s':44.1,'positions':[[1]*7],'internal_path':'private'}],
         'execution_gate_audits':[{'case':'stale','passed':True,'execute':0,
             'actual_camera_observation':False,'injected_observation':True,
             'error':'private details','raw_observation':{'pose':[1,2,3]}}]}
    row=pusht_summary(raw)
    assert row['complete_chain'] is False
    assert row['audited_stages_including_partial'][0]['samples']==1324
    assert row['execution_gate_audits'][0]['actual_camera_observation'] is False
    text=json.dumps(row)
    assert all(key not in text for key in ('private','positions','raw_observation'))


def test_return_endpoint_failure_remains_distinct_from_start_and_exports_no_joints(tmp_path):
    path=tmp_path/'events.jsonl'
    path.write_text(json.dumps({'event':'jimu_return_query_diagnostic','state_unchanged':True,
        'native_status':'TRAJOPT_FAIL','sim_qpos':[1]*15,'gripper_lock_joints':{'private':.6},
        'endpoints':{'start':{'valid':True,'diagnosed_q':[0]*7},
                     'goal':{'valid':False,'status':'WORLD_COLLISION','diagnosed_q':[1]*7,
                         'robot_world_obstacle_contacts':[{'robot_link':'left_pad','obstacle':'placed',
                             'clearance_m':-.003,'sphere_center':[1,2,3]}]}}}))
    row=event_summary(path,bare=True)['first_return_query_diagnostic']
    assert row['state_unchanged'] and row['endpoints']['start']['valid']
    assert not row['endpoints']['goal']['valid']
    assert row['endpoints']['goal']['negative_pairs'][0]['clearance_m']==-.003
    assert all(key not in json.dumps(row) for key in ('diagnosed_q','sim_qpos','sphere_center','private'))


def test_release_observation_preserves_unqualified_model_without_raw_joint_state(tmp_path):
    path=tmp_path/'events.jsonl'
    path.write_text(json.dumps({'event':'jimu_release_execution_observation','diagnostic_only':True,
        'execution_guard':False,'native_all_valid':True,'max_gripper_model_error_rad':.119,
        'table_present':False,'audited_samples':101,'sim_gripper_joints':{'private':.719}}))
    row=event_summary(path,bare=True)['release_execution_observations'][0]
    assert row['diagnostic_only'] and not row['execution_guard']
    assert row['native_all_valid'] and not row['table_present'] and row['max_gripper_model_error_rad']==.119
    assert 'private' not in json.dumps(row)


def test_release_invalid_evidence_exports_pairs_not_q_or_geometry(tmp_path):
    path=tmp_path/'events.jsonl'
    path.write_text(json.dumps({'event':'jimu_release_execution_observation','model_sync_requested':True,
        'first_invalid':[{'index':0,'status':'WORLD_COLLISION','geometry_detail_recorded':True,
            'diagnosed_q':[1]*7,'robot_world_obstacle_contacts':[{'robot_link':'pad','obstacle':'placed',
                'clearance_m':-.001,'sphere_center':[1,2,3]}]}]}))
    row=event_summary(path,bare=True)['release_execution_observations'][0]
    assert row['model_sync_requested'] and row['first_invalid'][0]['negative_pairs']==[
        {'robot_link':'pad','obstacle':'placed','clearance_m':-.001}]
    assert all(key not in json.dumps(row) for key in ('diagnosed_q','sphere_center'))
