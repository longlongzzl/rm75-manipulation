#!/usr/bin/env python3
"""Summarize persistent no-motion experiments without exporting native logs."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.io import atomic_json


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.input.resolve()
    report = dict(schema='rm75.pickplace_pusht_local/v1',
        tested_base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        hardware_connected=False, execute_real=False, verified_physical_success=None,
        raw_artifacts_local_only=True, pickplace=[], pusht=[])
    for path in sorted(root.glob('*/result.json')):
        name=path.parent.name; data=json.loads(path.read_text())
        log=root/(name+'.log'); raw=log.read_text(errors='replace') if log.exists() else ''
        common=dict(run=name,result_sha256=digest(path),log_sha256=digest(log) if log.exists() else None)
        if name.startswith('pickplace_'):
            warnings=[line for line in raw.splitlines()
                      if line.startswith('[warn] post-place clearance planning failed after release;')]
            audits=data.get('clearance_path_audits',[])
            report['pickplace'].append(dict(**common,case=data.get('case'),
                expected_cycles=data.get('expected_cycles'),command_success=data['command_success'],
                native_cycles=data.get('native_cycles'),native_final_success=data.get('native_final_success'),
                error=data.get('error'),clearance_failures=warnings,
                strict_clearance_success=bool(data['command_success'] and not warnings and
                    len(audits)==data.get('expected_cycles')),
                clearance_path_audits=audits,loaded_mplib_modules=data.get('loaded_mplib_modules')))
        elif name.startswith('gpu_'):
            records=data.get('backend_records',[])
            pairs=sorted({(str(c.get('collision_type')),str(c.get('robot_link')),
                          str(c.get('other_robot_link')),str(c.get('world_object')))
                for row in records for c in row.get('contacts',[])})
            report['pusht'].append(dict(**common,case=data['case'],fixture=data.get('fixture','raised_table'),
                scenario=data['scenario'],complete_chain=data['complete_chain'],error=data.get('error'),
                hardware_profile_qualified=data['hardware_profile_qualified'],elapsed_s=data['elapsed_s'],
                stages_audited=data.get('events',[]),
                plan_rows=[row for row in records if row['kind']=='plan'],
                collision_pairs=pairs,
                collision_audit_call_count=sum(r['kind']=='collision_audit' for r in records),
                native_ik_call_count=sum(r['kind']=='native_ik' for r in records)))
    report['cpu_checks']={}
    for key in ('three_scene_tests','full_tests'):
        path=root/(key+'.log'); raw=path.read_text()
        report['cpu_checks'][key]=dict(summary=raw.strip().splitlines()[-1],sha256=digest(path))
    for key in ('compileall','snapshot_verify'):
        path=root/(key+'.log')
        report['cpu_checks'][key]=dict(sha256=digest(path))
    report['snapshot']= {k:v for k,v in json.loads((root/'snapshot_verify.log').read_text()).items()
                        if k in ('files','source_commit','dependency_completeness_verified','runtime_gpu_verified')}
    report['browser']=json.loads((root/'browser/report.json').read_text())
    report['real_profile_qualification']=json.loads((root/'pusht_real_qualification/result.json').read_text())
    grid_path=root/'reachability_grid.log'
    grid=json.loads(next(line[len('GRID_RESULT '):] for line in grid_path.read_text().splitlines()
                         if line.startswith('GRID_RESULT ')))
    report['reachability_diagnostic']=dict(purpose=grid['purpose'],total=grid['candidates'],
        success_count=sum(row['success'] for row in grid['metrics'].values()),log_sha256=digest(grid_path),
        qualification='empty-world IK diagnostic only, not a full-chain pass')
    final=[row for row in report['pusht'] if row['run'].endswith('_final')]
    report['totals']=dict(pickplace_runs=len(report['pickplace']),
        pickplace_native_success=sum(row['command_success'] for row in report['pickplace']),
        pusht_gpu_runs=len(report['pusht']),pusht_gpu_success=sum(row['complete_chain'] for row in report['pusht']),
        pusht_final_matrix_cases=len(final),pusht_final_matrix_success=sum(row['complete_chain'] for row in final))
    report['notes']=[
        'gpu_low_orthogonal_v2 repeated pre-fix code because the patch executable path had expired.',
        'v3 used native linear scale 0.5; v4 used 1.0 but stopped at first violating waypoint.',
        'Final GPU cases use scale 1.0 and report the maximum error over all original path samples.',
        'Neighbor-blocked case rejected at IK; this is NOT proof of a completed collision-audit negative gate.',
        'Native PickPlace success is not physical success. Table-all v3 skipped failed shuazi clearance.',
        'Legacy PickPlace transport masks/scales were retained, not newly relaxed; full strict transport gate not claimed.',
    ]
    report['source_sha256']={name:digest(ROOT/name) for name in (
        'rm75_app/planning/backends/curobo2.py','rm75_app/pusht/motion.py',
        'rm75_app/workcell/pickplace_curobo_only.py',
        'rm75_app/workcell/pickplace_clearance_audit.py',
        'rm75_app/workcell/pickplace_gripper_state.py',
        'tools/run_native_pickplace.py','tools/run_pusht_gpu_validation.py',
        'tests/three_scene/test_native_pickplace_fixed_cases.py',
        'tests/three_scene/test_pickplace_return_transaction.py',
        'tests/three_scene/test_pusht_motion_chain.py')}
    atomic_json(args.output,report)
    print(json.dumps(report['totals']))


if __name__=='__main__': main()
