#!/usr/bin/env python3
"""Export bounded no-motion matrix evidence, never raw paths or trajectories."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.io import atomic_json
from tools.run_pusht_gpu_validation import validation_passed
from tools.summarize_native_gaps import lift_ik_summary

EXPECTED_RUNS = (
    'gpu_orthogonal', 'gpu_orthogonal_slow', 'gpu_baseline',
    'gpu_rotated_orthogonal', 'gpu_yaw_matched_tool', 'gpu_orthogonal_neighbor_blocked',
    'pickplace_legacy_gluestick', 'pickplace_gluestick_jitter_00',
    'pickplace_gluestick_yaw_00', 'pickplace_gluestick_swap_00', 'pickplace_current_table_all',
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def select(data, keys):
    return {key: data.get(key) for key in keys}


def snapshot_file_count(value):
    # verify-only reports a count; a full manifest instead contains file rows.
    count = len(value) if isinstance(value, list) else value
    if type(count) is not int or count < 1:
        raise ValueError('Invalid verified snapshot file count')
    return count


def compact_lift_diagnostic(raw):
    row = lift_ik_summary(raw)
    returned = row.pop('returned_rows')
    statuses = [str(r['configuration']['native_status']) for r in returned]
    row['returned_row_count'] = len(returned)
    row['native_success_rows'] = sum(r['native_success'] is True for r in returned)
    row['returned_status_counts'] = {s: statuses.count(s) for s in sorted(set(statuses))}
    finite = [r for r in returned if isinstance(r['position_error_m'], (int, float))
              and math.isfinite(r['position_error_m'])]
    row['lowest_position_error_return'] = min(finite, key=lambda r: r['position_error_m']) if finite else None
    return row


def summarize_run(name, data):
    if data.get('execute_real') is not False or data.get('hardware_connected', False) is not False:
        raise ValueError('Only explicitly non-real results may enter this matrix')
    row = dict(run=name, case=data['case'], elapsed_s=data['elapsed_s'])
    if name.startswith('gpu_'):
        stages = [dict(**select(s, ('stage', 'max_tcp_speed_mps',
            'max_joint_speed_rad_s', 'max_joint_accel_rad_s2')),
            samples=len(s['positions']), duration_s=s['times'][-1])
            for s in data.get('stages', [])]
        row.update(kind='pusht_gpu', complete_chain=data['complete_chain'],
            validation_success=validation_passed(data, audit_blocker=True, audit_gates=True),
            reported_validation_success=data.get('validation_success'),
            hardware_profile_qualified=data.get('hardware_profile_qualified'),
            fixture=data.get('fixture'), requested_speed_mps=data['config']['speed_mps'],
            stages=stages, total_samples=sum(s['samples'] for s in stages),
            total_duration_s=sum(s['duration_s'] for s in stages),
            unrelated_obstacle_rejected=data.get('unrelated_obstacle_audit', {}).get('rejected'),
            execution_gates=[select(g, ('case', 'passed', 'execute', 'injected_observation',
                'actual_camera_observation')) for g in data.get('execution_gate_audits', [])])
        error = data.get('error') or ''
        row['error_type'] = error.split(':', 1)[0] if error else None
        match = re.search(r'waypoint=(\d+)/(\d+)', error)
        row['failed_waypoint'] = [int(x) for x in match.groups()] if match else None
        row['failed_stage'] = ('descend' if 'pusht:descend' in error else
                               'approach' if 'at approach' in error else None)
    elif name.startswith('pickplace_'):
        audits = data.get('clearance_path_audits', [])
        transport = data.get('transport_path_audits', [])
        row.update(kind='pickplace_native', **select(data, ('expected_cycles', 'command_success',
            'native_final_success', 'native_completed_cycles', 'loaded_mplib_modules')),
            cycle_attempts=len(data.get('native_cycles', [])),
            cycle_successes=sum(c.get('success') is True for c in data.get('native_cycles', [])),
            clearance_warning_count=len(data.get('clearance_failures', [])),
            clearance_audits=[select(a, ('stage', 'samples', 'all_valid', 'world_exempt_links'))
                              for a in audits],
            transport_audits=[select(a, ('samples', 'candidate_paths', 'payload_spheres',
                'world_exempt_links', 'scene_fingerprint')) for a in transport],
            diagnostic_rejection_count=len(data.get('diagnostic_rejections', [])))
        row['lift_ik_diagnostics'] = [compact_lift_diagnostic(d) for d in data.get('lift_ik_diagnostics', [])]
        expected = data.get('expected_cycles', 0)
        row['strict_clearance_success'] = bool(expected > 0 and
            data.get('strict_clearance_success') is True and data.get('command_success') is True and
            data.get('native_final_success') is True and not row['clearance_warning_count'] and
            len(data.get('native_completed_cycles', [])) == expected and len(audits) == expected and
            all(a.get('all_valid') is True and a.get('samples', 0) > 0 for a in audits))
    else:
        raise ValueError('Unknown matrix run')
    return row


def totals(rows):
    gpu = [r for r in rows if r['kind'] == 'pusht_gpu']
    native = [r for r in rows if r['kind'] == 'pickplace_native']
    return dict(pusht_gpu_runs=len(gpu), pusht_complete_chains=sum(r['complete_chain'] for r in gpu),
        pusht_validation_passes=sum(r['validation_success'] for r in gpu),
        pusht_normal_runs=sum(r['case'] != 'orthogonal_neighbor_blocked' for r in gpu),
        pusht_preset_obstacle_runs=sum(r['case'] == 'orthogonal_neighbor_blocked' for r in gpu),
        post_plan_obstacle_rejections=sum(r['unrelated_obstacle_rejected'] is True for r in gpu),
        injected_execution_gates=sum(len(r['execution_gates']) for r in gpu),
        injected_execution_gate_passes=sum(g['passed'] is True for r in gpu for g in r['execution_gates']),
        execution_calls=sum(g['execute'] for r in gpu for g in r['execution_gates']),
        pickplace_runs=len(native),
        pickplace_strict_clearance_passes=sum(r['strict_clearance_success'] for r in native))


def build_report(root):
    missing = [name for name in EXPECTED_RUNS if not (root/name/'result.json').is_file()]
    if missing:
        raise ValueError(f'Matrix incomplete: {missing}')
    rows = []
    extra = sorted(p.parent.name for p in root.glob('*/result.json')
        if p.parent.name.startswith(('gpu_', 'pickplace_')) and p.parent.name not in EXPECTED_RUNS)
    for name in (*EXPECTED_RUNS, *extra):
        path = root/name/'result.json'
        row = summarize_run(name, json.loads(path.read_text()))
        row.update(result_sha256=digest(path), log_sha256=digest(root/(name+'.log')))
        row['run_purpose'] = 'fixed_case_matrix' if name in EXPECTED_RUNS else 'diagnostic_followup'
        rows.append(row)
    report = dict(schema='rm75.no_motion_matrix/v1',
        tested_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        hardware_connected=False, execute_real=False, verified_physical_success=None,
        raw_artifacts_local_only=True, runs=rows, totals=totals(rows))
    report['fixed_matrix_totals'] = totals([r for r in rows if r['run_purpose'] == 'fixed_case_matrix'])
    report['cpu_checks'] = {name: dict(sha256=digest(root/(name+'.log')),
        summary=(root/(name+'.log')).read_text().strip().splitlines()[-1])
        for name in ('three_scene_tests', 'full_tests')}
    snapshot = json.loads((root/'snapshot_verify.log').read_text())
    report['snapshot'] = dict(**select(snapshot, ('source_commit',
        'dependency_completeness_verified', 'runtime_gpu_verified')),
        files=snapshot_file_count(snapshot['files']))
    report['cli'] = {}
    for name in ('pickplace_preview', 'magnetic_preview', 'pusht_preview', 'pusht_sim'):
        path = root/('cli_'+name+'.log'); data = json.loads(path.read_text())
        report['cli'][name] = dict(status=data['status'], sha256=digest(path),
            result=select(data['result'], ('command_success', 'task_success', 'verification',
                'mode', 'steps', 'success_observations', 'model_validated_on_robot')))
    qualification = json.loads((root/'real_profile_qualification/result.json').read_text())
    report['real_profile_qualification'] = dict(**select(qualification, ('status',
        'execute_real', 'hardware_connected', 'gpu_chain_planned', 'missing_run_inputs')),
        qualification=select(qualification['qualification'], ('hardware_reviewed',
            'integration_qualified', 'motion_authorized', 'planning_inputs_complete')),
        missing_fields=[e['field'] for e in qualification['qualification']['errors']])
    report['http_assets'] = {suffix: dict(sha256=digest(root/('workcell.'+suffix)),
        bytes=(root/('workcell.'+suffix)).stat().st_size) for suffix in ('html', 'css', 'js')}
    warm = json.loads((root/'relation_warm_summary.json').read_text())
    report['unified_relation_screen'] = select(warm, ('planner_construction_s', 'sample_count',
        'warm_sample_count', 'cold_first_call_s', 'warm_relation_screen_wall_time_s',
        'relation_found_rate', 'tasks', 'mean_coarse_ik_calls', 'mean_coarse_ik_rows_requested',
        'mean_coarse_ik_rows_padded'))
    report['unified_relation_screen']['raw_sha256'] = digest(root/'relation_warm.jsonl')
    full_path = root/'relation_full_chain.jsonl'
    full = [json.loads(line) for line in full_path.read_text().splitlines() if line.strip()]
    report['unified_full_chain'] = dict(raw_sha256=digest(full_path),
        executor='no_op_not_physics_or_native_runtime', runs=len(full),
        successes=sum(r['full_chain_plan_success'] is True for r in full),
        rows=[select(r, ('plan_id', 'atom_id', 'object_id', 'full_chain_plan_success',
            'failure_stage', 'executed_stage_names', 'relation_screen_mode',
            'grasp_reverse_fallback_used', 'grasp_tool_axis_retry_used',
            'grasp_candidate_count', 'place_candidate_count', 'declared_place_candidate_count',
            'coarse_ik_rows_requested', 'coarse_ik_rows_padded')) for r in full])
    report['source_sha256'] = {name: digest(ROOT/name) for name in (
        'tools/summarize_no_motion_matrix.py', 'tests/three_scene/test_no_motion_matrix_summary.py',
        'tools/run_native_pickplace.py', 'tools/run_pusht_gpu_validation.py',
        'tools/benchmark_grasp_relation_screen.py', 'rm75_app/workcell/pickplace_curobo_only.py',
        'rm75_app/workcell/pickplace_lift_diagnostics.py', 'rm75_app/workcell/transport_contact.py',
        'rm75_app/pusht/motion.py', 'rm75_app/planning/backends/curobo2.py',
        'benchmarks/task001/fixtures/d0_cube_lvmukuai/manipulation_plan.json',
        'benchmarks/task001/fixtures/d0_asym_shuazi/manipulation_plan.json')}
    report['preexisting_dirty_files_sha256'] = {name: digest(ROOT/name) for name in (
        'rm75_app/workcell/legacy.py', 'tools/run_native_contact_audit.py',
        'tools/run_workcell_native_validation.py', 'rm75_app/workcell/jimu_roof_ik_diagnostics.py',
        'tests/three_scene/test_jimu_roof_ik_diagnostics.py')}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = build_report(args.input)
    atomic_json(args.output, report)
    print(json.dumps(report['totals']))


if __name__ == '__main__':
    main()
