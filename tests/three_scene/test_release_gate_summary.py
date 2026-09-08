import hashlib
import json
import pytest

from tools import summarize_release_gate as summary


def test_native_summary_keeps_failed_attempts_and_excludes_raw_data(tmp_path):
    directory = tmp_path / 'standard_roof'
    directory.mkdir()
    raw = {'validation_success': False, 'expected_cycles': 12,
           'native_completed_cycles': list(range(1, 8)), 'native_final_success': False,
           'native_cycles': [{'cycle': i, 'success': True} for i in range(1, 8)] +
                            [{'cycle': 8, 'success': False}] * 5,
           'argv': ['/PRIVATE_PATH'], 'joints': [987654321.0],
           'camera_serial': 'PRIVATE_HARDWARE_ID', 'loaded_mplib_modules': []}
    path = directory / 'result.json'
    path.write_text(json.dumps(raw))
    (directory / 'contact.jsonl').write_text(json.dumps({
        'event': 'transport_full_world_audit', 'samples': 42,
        'payload_spheres': 6, 'world_exempt_links': [], 'joints': [987654321.0]}) + '\n')
    result = summary.native_run(path)
    assert not result['completed']
    assert result['native_successful_cycles'] == 7
    assert result['native_cycle_attempts'] == 12
    assert result['native_failed_attempts'] == 5
    assert result['transport_world_checked']
    assert result['result_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    encoded = json.dumps(result)
    assert all(secret not in encoded for secret in ('PRIVATE_PATH', 'PRIVATE_HARDWARE_ID', '987654321'))
    assert result['physical_task_success'] is None


def test_worker_summary_preserves_requested_source_and_prefetch_scope(tmp_path, monkeypatch):
    monkeypatch.setattr(summary, 'ROOT', tmp_path)
    job = tmp_path / 'runtime_data/workcell/jobs/local-id'
    job.mkdir(parents=True)
    (job / 'events.jsonl').write_text('')
    path = tmp_path / 'result.json'
    path.write_text(json.dumps({'job_id': 'local-id', 'completed': False,
        'native_final_success': True, 'job': {'status': 'failed', 'result': {
            'frozen_world_validation': {'requested_sources': ['gluestick'],
                'requested_sources_completed': False, 'source_outcomes': [
                    {'source': 'bi', 'success': True, 'foreground': True, 'prefetch_capture_only': False},
                    {'source': 'lvmukuai', 'success': True, 'foreground': False, 'prefetch_capture_only': True}]}}}}))
    result = summary.native_run(path)
    assert not result['completed'] and result['native_final_success']
    assert not result['frozen_world_validation']['requested_sources_completed']
    assert result['source_outcomes'][1]['prefetch_capture_only'] is True
    assert not result['transport_world_checked']
    assert 'local-id' not in json.dumps(result)


def test_grasp_summary_excludes_joints_poses_and_preserves_failure():
    row={'source':'gluestick','native_success_count':0,'goals':[{
        'goal_index':0,'native_success':False,'nearest_seed_index':0,
        'seeds':[{'finite':True,'position_error_m':.00002,'joints':[987654321.]}],
        'configuration':{'joints':[987654321.],'fk':{'position':[987654321.]},
            'valid':False,'native_status':'WORLD_COLLISION',
            'box_contacts':[{'link':'right_pad','obstacle':'shuazi','overlap_m':.006,'enabled':True}]}}]}
    result=summary.grasp_summary(row)
    assert result['native_success_count']==0 and result['goals'][0]['valid'] is False
    assert result['goals'][0]['box_contacts'][0]['obstacle']=='shuazi'
    assert '987654321' not in json.dumps(result)


def test_camera_count_requires_explicit_manual_review_and_excludes_device_id(tmp_path):
    (tmp_path / 'camera_availability.json').write_text(json.dumps({
        'device_count': 1, 'device_models': ['D435'], 'camera_stream_started': False,
        'robot_connected': False, 'serial': 'PRIVATE_CAMERA_SERIAL'}))
    path = tmp_path / 'camera_overview/rgb/000000.png'
    path.parent.mkdir(parents=True)
    path.write_bytes(b'fixture')
    result = summary.camera_summary(tmp_path)
    assert result['manually_confirmed_matching_pieces_in_overview'] is None
    assert result['piece_count_source'] == 'NOT_REVIEWED'
    assert result['camera_capture_performed'] is True
    assert result['initial_availability_probe']['camera_stream_started'] is False
    assert result['initial_availability_probe']['device_models'] == ['D435']
    assert 'PRIVATE_CAMERA_SERIAL' not in json.dumps(result)
    reviewed = summary.camera_summary(tmp_path, 1)
    assert reviewed['manually_confirmed_matching_pieces_in_overview'] == 1
    assert reviewed['piece_count_source'] == 'manual_image_review'
    assert reviewed['multi_instance_rrtrack_status'] == 'NOT_RUN'
    with pytest.raises(ValueError, match='nonnegative'):
        summary.camera_summary(tmp_path, -1)
