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
