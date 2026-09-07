from copy import deepcopy
import json

import pytest

from tools.summarize_no_motion_matrix import (
    build_report, compact_lift_diagnostic, snapshot_file_count, summarize_run, totals,
)
from tools.run_pusht_gpu_validation import EXECUTION_GATE_CASES


def native():
    return dict(execute_real=False, case='fixture', elapsed_s=1., expected_cycles=1,
        command_success=True, native_final_success=True, strict_clearance_success=True,
        native_completed_cycles=[1], native_cycles=[dict(cycle=1, success=True)],
        clearance_path_audits=[dict(stage='clearance', samples=5, all_valid=True,
            world_exempt_links=[], positions=[[999]])], clearance_failures=[],
        argv=['/private/source'], traceback='/private/trace', loaded_mplib_modules=[])


@pytest.mark.parametrize('value,expected', [(807, 807), ([{}], 1),
    (True, None), (-1, None), (0, None), (None, None)])
def test_verify_count_and_manifest_rows_are_not_confused(value, expected):
    if expected is None:
        with pytest.raises(ValueError, match='snapshot file count'):
            snapshot_file_count(value)
    else:
        assert snapshot_file_count(value) == expected


def test_native_export_is_bounded_and_does_not_inherit_trajectories_or_paths():
    data = native(); before = deepcopy(data)
    row = summarize_run('pickplace_fixture', data)
    assert row['strict_clearance_success'] is True
    assert row['cycle_attempts'] == row['cycle_successes'] == 1
    assert 'private' not in json.dumps(row) and 'positions' not in json.dumps(row)
    assert data == before


@pytest.mark.parametrize('change', [
    {'clearance_failures': ['warning']}, {'native_final_success': False},
    {'command_success': False}, {'native_completed_cycles': []},
    {'clearance_path_audits': []}, {'strict_clearance_success': False},
    {'clearance_path_audits': [dict(samples=5, all_valid=False)]},
])
def test_native_success_cannot_hide_incomplete_or_unsafe_outcome(change):
    data = native(); data.update(change)
    assert summarize_run('pickplace_fixture', data)['strict_clearance_success'] is False


@pytest.mark.parametrize('change', [{'execute_real': True}, {'execute_real': None},
                                  {'hardware_connected': True}])
def test_non_sim_results_refused(change):
    data = native(); data.update(change)
    with pytest.raises(ValueError, match='explicitly non-real'):
        summarize_run('pickplace_fixture', data)


def test_gpu_failure_remains_in_denominator_and_private_error_is_not_exported():
    data = dict(execute_real=False, hardware_connected=False, case='baseline', elapsed_s=2.,
        complete_chain=False, validation_success=False, config={'speed_mps': .015},
        error='PushPathRejected: cartesian_ik_failed:pusht:descend: waypoint=10/16 /private/raw')
    row = summarize_run('gpu_baseline', data)
    assert row['failed_waypoint'] == [10, 16] and row['failed_stage'] == 'descend'
    assert 'private' not in json.dumps(row)
    counts = totals([row, summarize_run('pickplace_fixture', native())])
    assert counts['pusht_gpu_runs'] == 1 and counts['pusht_validation_passes'] == 0
    assert counts['pickplace_runs'] == counts['pickplace_strict_clearance_passes'] == 1


def test_missing_run_is_not_silently_removed_from_matrix(tmp_path):
    with pytest.raises(ValueError, match='Matrix incomplete'):
        build_report(tmp_path)


def test_lift_followup_keeps_failure_counts_not_raw_solver_arrays():
    row = compact_lift_diagnostic(dict(goal_pose=[999], returned_rows=[
        dict(index=0, native_success=False, position_error_m=.001, q=[999],
            configuration={'native_status': 'SELF_COLLISION'}),
        dict(index=1, native_success=False, position_error_m=.1, q=[998],
            configuration={'native_status': 'WORLD_COLLISION'})]))
    assert row['returned_row_count'] == 2 and row['native_success_rows'] == 0
    assert row['returned_status_counts'] == {'SELF_COLLISION': 1, 'WORLD_COLLISION': 1}
    assert row['lowest_position_error_return']['index'] == 0
    assert '999' not in json.dumps(row) and 'goal_pose' not in row


@pytest.mark.parametrize('failure', [None, 'dynamics', 'gates', 'obstacle'])
def test_complete_gpu_chain_still_requires_all_requested_audits(failure):
    data = dict(execute_real=False, hardware_connected=False, case='orthogonal_tool',
        elapsed_s=1., complete_chain=True, validation_success=True,
        config={'speed_mps': .015}, motion={'joint_speed_rad_s': .25, 'joint_accel_rad_s2': .5},
        stages=[dict(stage=s, positions=[[999], [998]], times=[0., 1.], max_tcp_speed_mps=.01,
            max_joint_speed_rad_s=.1, max_joint_accel_rad_s2=.2)
            for s in ('approach', 'descend', 'contact', 'push', 'retreat')],
        unrelated_obstacle_audit={'rejected': True},
        execution_gate_audits=[dict(case=c, passed=True, execute=0) for c in EXECUTION_GATE_CASES])
    if failure == 'dynamics':
        data['stages'][0]['max_tcp_speed_mps'] = .02
    elif failure == 'gates':
        data['execution_gate_audits'].pop()
    elif failure == 'obstacle':
        data['unrelated_obstacle_audit']['rejected'] = False
    row = summarize_run('gpu_orthogonal', data)
    assert row['complete_chain'] is True
    assert row['validation_success'] is (failure is None)
    assert row['total_samples'] == 10 and row['total_duration_s'] == 5
    assert '999' not in json.dumps(row)
