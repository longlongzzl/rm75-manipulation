#!/usr/bin/env python3
"""Bounded release-gate evidence; never export trajectories, joints or device IDs."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot
from tools.summarize_native_gaps import lift_ik_summary, roof_ik_summary


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pick(data, fields):
    return {key: data.get(key) for key in fields.split()}


def grasp_summary(row):
    result = pick(row, 'source phase diagnostic_only new_solver_calls goal_count '
        'requested_num_seeds native_success_count diagnostic_complete state_unchanged returned_rows_unchanged error_type')
    result['goals'] = []
    for goal in row.get('goals', []):
        seed = goal['seeds'][goal['nearest_seed_index']]
        configuration = goal.get('configuration', {})
        result['goals'].append({**pick(goal, 'goal_index native_success'),
            'nearest_seed': pick(seed, 'finite native_success position_error_m rotation_error_native'),
            **pick(configuration, 'valid native_status'),
            'box_contacts': [pick(r, 'link obstacle overlap_m enabled') for r in configuration.get('box_contacts', [])],
            'self_pairs': [pick(r, 'link_a link_b overlap count') for r in
                configuration.get('self_collision', {}).get('link_pairs', [])]})
    return result


def camera_summary(directory, visually_confirmed_pieces=None):
    if visually_confirmed_pieces is not None and visually_confirmed_pieces < 0:
        raise ValueError('Manual piece count must be nonnegative')
    path = directory / 'camera_availability.json'
    if not path.exists():
        return None
    probe = json.loads(path.read_text())
    result = {'availability_sha256': sha(path),
        'initial_availability_probe': pick(probe, 'device_count d435_count device_models camera_stream_started robot_connected'),
        'manually_confirmed_matching_pieces_in_overview': visually_confirmed_pieces,
        'piece_count_source': 'manual_image_review' if visually_confirmed_pieces is not None else 'NOT_REVIEWED',
        'camera_capture_performed': False,
        'multi_instance_rrtrack_status': 'NOT_RUN',
        'controlled_occlusion_status': 'NOT_RUN', 'base_frame_pose_spread_status': 'NOT_RUN'}
    overview = directory / 'camera_overview/rgb/000000.png'
    if overview.exists():
        result.update(overview_sha256=sha(overview), camera_capture_performed=True)
    clip = directory / 'camera_review_clip_5fps.mp4'
    capture = directory / 'camera_review_clip/capture.jsonl'
    if clip.exists() and capture.exists():
        rows = [json.loads(line) for line in capture.read_text().splitlines()]
        if not rows:
            raise ValueError('Review video has no capture metadata')
        result['camera_capture_performed'] = True
        result['review_video'] = {'sha256': sha(clip), 'bytes': clip.stat().st_size,
            'capture_metadata_sha256': sha(capture), 'frames': len(rows), 'encoding_fps': 5,
            'host_record_timestamp_span_s': rows[-1]['timestamp_s']-rows[0]['timestamp_s'],
            'device_timestamp_span_ms': rows[-1]['device_timestamp_ms']-rows[0]['device_timestamp_ms'],
            'hardware_identifier_exported': False, 'robot_connected': False,
            'identity_or_physical_success_qualified': False}
    return result


def native_run(path):
    raw = json.loads(path.read_text())
    worker = bool(raw.get('job_id'))
    result = raw.get('job', {}).get('result', {}) if worker else raw
    local = ROOT / 'runtime_data/workcell/jobs' / raw['job_id'] if worker else path.parent
    events = local / ('events.jsonl' if worker else 'contact.jsonl')
    rows = []
    if events.exists():
        for line in events.read_text().splitlines():
            item = json.loads(line)
            if not worker or item.get('kind') == 'contact_audit':
                rows.append(item['evidence'] if worker else item)
    transport = [r for r in rows if r.get('event') == 'transport_full_world_audit']
    release = [r for r in rows if r.get('event') == 'jimu_release_execution_audit']
    roof = []
    for row in rows:
        if row.get('event') != 'jimu_roof_ik_batch_diagnostic':
            continue
        summary = roof_ik_summary(row)
        selected = pick(summary, 'source phase prefetch goal_count native_success_count '
            'requested_num_seeds diagnostic_complete state_unchanged raw_success_rows '
            'debug_nearest_error_mismatch_count')
        selected['nominal_gripper_base_boxes'] = summary.get('nominal_gripper_base_boxes')
        selected['failed_nearest_position_error_m'] = summary.get('failed_nearest_position_error_m')
        roof.append(selected)
    frozen = result.get('frozen_world_validation', {})
    failed_world = [r for r in rows if r.get('event') == 'jimu_read_only_collision_diagnostic']
    report = {
        'run': path.parent.name,
        'result_sha256': sha(path),
        'actual_workcell_worker': worker,
        'completed': raw.get('completed', raw.get('validation_success', False)),
        **pick(raw, 'elapsed_s timeout expected_cycles native_final_success native_full_chain_passed'),
        'native_successful_cycles': len(raw.get('native_completed_cycles', [])),
        'native_cycle_attempts': len(raw.get('native_cycles', [])),
        'native_failed_attempts': sum(r.get('success') is False for r in raw.get('native_cycles', [])),
        'worker_status': raw.get('job', {}).get('status'),
        'failure_code': result.get('failure_code'),
        'terminal_contact_rejection': pick(result.get('contact_evidence', {}),
            'event reason step_id disabled_links disabled_objects'),
        'error_type': str(raw.get('error', '')).split(':', 1)[0],
        'clearance_failure_count': len(raw.get('clearance_failures', [])),
        'loaded_mplib_modules': result.get('loaded_mplib_modules'),
        'original_task_bundle_sha256': raw.get('original_task_bundle_sha256'),
        'original_task_bundle_unchanged': raw.get('original_task_bundle_unchanged'),
        'documented_start_source_unchanged': raw.get('documented_start_source_unchanged'),
        'event_counts': dict(Counter(r.get('event') for r in rows)),
        'transport_audits': [pick(r, 'step_id samples payload_spheres world_exempt_links') for r in transport],
        'transport_samples': sum(r.get('samples', 0) for r in transport),
        'transport_world_checked': bool(transport) and all(r.get('world_exempt_links') == [] for r in transport),
        'release_execution_audits': [pick(r, 'source passed state_unchanged stage_kind clearance_samples '
            'return_samples return_world_exempt_links release_contact_samples error_type') for r in release],
        'roof_ik_audit': raw.get('roof_ik_audit'),
        'roof_batches': roof,
        'first_transport_world_diagnostics': [{**pick(r,
            'step_id status valid diagnosed_q_role geometry_detail_recorded diagnostic_mode'),
            'disabled_links': r.get('disabled_links'),
            'native_nonzero_links': [pick(item, 'link value') for item in
                r.get('curobo_raw_world_collision', {}).get('nonzero', [])],
            'negative_cuboid_pairs': [pick(item, 'robot_link obstacle clearance_m') for item in
                r.get('robot_world_obstacle_contacts', []) if item.get('clearance_m', 0) < 0],
            'mesh_obstacle_identity_qualified': False} for r in failed_world[:3]],
        'lift_diagnostics': [lift_ik_summary(r) for r in rows
                             if r.get('event') == 'pickplace_failed_lift_ik_diagnostic'],
        'grasp_batch_diagnostics': [grasp_summary(r) for r in rows
                                    if r.get('event') == 'pickplace_grasp_batch_diagnostic'],
        'existing_reverse_diagnostics': [{**pick(r, 'source diagnostic_only selected new_solver_calls '
            'path_points accepted_by_same_full_path_gate state_unchanged error_type reason'),
            'audit': pick(r.get('audit', {}), 'samples audited_samples all_valid world_exempt_links '
                'initial_penetration_m final_penetration_m contact_sample_count '
                'monotonic_contact_nonincreasing max_step_penetration_increase_m'),
            'eligible_for_monotonic_reuse': bool(r.get('state_unchanged') and
                r.get('accepted_by_same_full_path_gate') and
                r.get('audit', {}).get('monotonic_contact_nonincreasing') is True)} for r in rows
            if r.get('event') == 'pickplace_existing_reverse_diagnostic'],
        'frozen_world_validation': pick(frozen, 'passed frozen_input_sha256 input_unchanged frozen_names '
            'requested_sources requested_sources_completed all_sources_transport_observed independent_clearance_passed '
            'refresh_count foreground_refresh_count foreground_transport_refresh_count physical_geometry_qualified'),
        'source_outcomes': [pick(r, 'source success foreground prefetch_capture_only') for r in frozen.get('source_outcomes', [])],
        'rejected_world_refresh_count': len(frozen.get('rejected_refreshes', [])),
        'clearance_selection_audits': [pick(r, 'kind accepted reason path_points samples audited_samples '
            'initial_penetration_m final_penetration_m contact_sample_count world_exempt_links')
            for r in result.get('clearance_selection_audits', [])],
        'clearance_execution_audits': [pick(r, 'samples audited_samples all_valid world_exempt_links '
            'initial_penetration_m final_penetration_m contact_sample_count')
            for r in result.get('clearance_path_audits', [])],
        'physical_task_success': None,
    }
    if events.exists():
        report['events_sha256'] = sha(events)
    log = local / 'stdout.log' if worker else path.parent.parent / (path.parent.name + '.log')
    if log.exists():
        report['stdout_sha256'] = sha(log)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--visually-confirmed-jimu-pieces', type=int,
        help='Explicit manual overview count, not an RRTrack identity result')
    args = parser.parse_args()
    directory = args.input.resolve()
    snapshot = ROOT / 'rm75_app/_vendor/working_snapshot'
    manifest = verify_snapshot(snapshot)
    overlay = json.loads((ROOT / 'configs/workcell/approved_worktree_overlay_20260907.json').read_text())
    runs = []
    for path in sorted(directory.glob('*/result.json')):
        if path.parent.name in ('four_wall', 'standard_roof', 'original_builder') or path.parent.name.startswith('pickplace_'):
            runs.append(native_run(path))
    report = {
        'schema': 'rm75.three_scene_release_gate/v1',
        'tested_base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'execute_real': False, 'robot_hardware_connected': False, 'physical_task_success': None,
        'raw_artifacts_local_only': True,
        'migration': {
            'fixed_source_commit': manifest['source_commit'], 'verified_file_count': manifest['file_count'],
            'manifest_sha256': sha(snapshot / 'MIGRATION_MANIFEST.json'),
            'single_overlay_file_count': len(overlay['files']),
            'approved_archive_paths': [r['path'] for r in manifest['source_worktree_overlay']['files']
                                        if r.get('byte_source') == 'approved_archive'],
            'new_assets': [pick(r, 'path sha256') for r in overlay['files'] if r['path'].startswith('Demo_Triangle/')],
            'runtime_gpu_verified': manifest['runtime_gpu_verified'],
            'dependency_completeness_verified': manifest['dependency_completeness_verified'],
        },
        'runs': runs,
        'checks': {},
        'notes': [
            'Current-table requests are independent frozen-world resets, not a sequential clearing task.',
            'All retries and intermediate failures are retained; no alternate-source or prefetch success substitutes for a request.',
            'SIM native completion is not physical assembly, grasp, tracking, or SDK qualification.',
            'Transport audit counts may include background planning; they are not executed-object counts.',
            'source_sha256 describes the final export working tree, not the source revision of every earlier run.',
        ],
    }
    for path in sorted(directory.glob('*tests.log')):
        lines = path.read_text().splitlines()
        report['checks'][path.name] = {'sha256': sha(path), 'last_line': lines[-1] if lines else None}
    for path in sorted(directory.glob('*compile.log')):
        report['checks'][path.name] = {'sha256': sha(path), 'bytes': path.stat().st_size}
    for task in ('pickplace', 'magnetic'):
        path = directory / (task + '_contract.json')
        if path.exists():
            raw = json.loads(path.read_text())
            report['checks'][task + '_contract'] = {'sha256': sha(path), **pick(raw,
                'native_main_called planner_called robot_control_called missing_required_flags')}
    path = directory / 'restored_triangle_geometry.json'
    if path.exists():
        geometry = json.loads(path.read_text())
        report['restored_triangle_geometry'] = {'sha256': sha(path), **pick(geometry,
            'compared_asset_selection_equivalent old_worktree_and_task_unchanged old_status_sha256_before '
            'old_status_sha256_after original_task_hashes environments original_collision_cache')}
    path = directory / 'pusht_physical_profile/result.json'
    if path.exists():
        raw = json.loads(path.read_text())
        report['pusht_physical_profile'] = {'sha256': sha(path),
            'profile_is_unqualified_template_not_measured_hardware': True,
            'hardware_profile_qualified': False, 'actual_tool_selection_confirmed': False,
            **pick(raw, 'status execute_real hardware_connected gpu_chain_planned missing_run_inputs'),
            'qualification': raw.get('qualification')}
    camera = camera_summary(directory, args.visually_confirmed_jimu_pieces)
    if camera is not None:
        report['camera'] = camera
    changed = subprocess.check_output(['git', 'diff', '--name-only'], cwd=ROOT, text=True).splitlines()
    changed += subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard'], cwd=ROOT, text=True).splitlines()
    report['source_sha256'] = {name: sha(ROOT / name) for name in sorted(set(changed)) if name.endswith('.py')}
    atomic_json(args.output, report)
    print(json.dumps({'runs': len(runs), 'summary_sha256': sha(args.output)}))


if __name__ == '__main__':
    main()
