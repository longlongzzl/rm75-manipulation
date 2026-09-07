#!/usr/bin/env python3
"""Summarize local native worker/transport evidence, without uploading raw data."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from tools.run_workcell_native_validation import native_outcomes


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.input.resolve()
    report={'schema':'rm75.native_closeout_followup/v1',
        'tested_base_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'execute_real':False,'hardware_connected':False,'physical_success':None,
        'raw_artifacts_local_only':True,'pickplace':[],'native_workers':[],'browser':[]}
    for path in sorted(root.glob('*/result.json')):
        raw=json.loads(path.read_text());name=path.parent.name
        log=root/(name+'.log')
        common={'run':name,'result_sha256':sha(path),'log_sha256':sha(log) if log.is_file() else None}
        if raw.get('planner')=='curobo_only':
            fields=('case','expected_cycles','command_success','native_cycles','native_final_success',
                    'clearance_failures','strict_clearance_success','clearance_path_audits',
                    'transport_path_audits','diagnostic_rejections','loaded_mplib_modules')
            error=raw.get('error','')
            # Early typed-policy exceptions embed the entire collision world;
            # retain the explicit code/reasons without exporting that raw world.
            if raw.get('transport_policy_rejections'):error=error.split(':',1)[0]
            report['pickplace'].append({**common,**{k:raw.get(k) for k in fields},'error':error,
                'policy_rejections':[{k:row.get(k) for k in ('reason','step_id')}
                    for row in raw.get('transport_policy_rejections',[])]})
        elif raw.get('job_id'):
            job_dir=ROOT/'runtime_data/workcell/jobs'/raw['job_id']
            stdout=job_dir/'stdout.log';events=job_dir/'events.jsonl'
            text=stdout.read_text(errors='replace')
            rows=[json.loads(line) for line in events.read_text().splitlines()]
            evidence=[row['evidence'] for row in rows if row.get('kind')=='contact_audit']
            audits=[row for row in evidence if row.get('event')=='transport_full_world_audit' and row.get('samples',0)>0]
            diagnostics=[row for row in evidence if row.get('event')=='jimu_read_only_collision_diagnostic']
            data=raw.get('job',{}).get('result',{})
            expected=raw.get('expected_cycles',12 if raw['task']=='magnetic' else 1)
            outcome=native_outcomes(text,expected)
            report['native_workers'].append({**common,**outcome,'task':raw['task'],
                'job_id':raw['job_id'],'worker_stdout_sha256':sha(stdout),'worker_events_sha256':sha(events),
                'recorded_completed':raw.get('completed'),'worker_status':raw.get('job',{}).get('status'),
                'worker_error':data.get('error'),'elapsed_s':raw.get('elapsed_s'),
                'native_success_is_not_physical_success':True,
                'stopped_for_validation':raw.get('stopped_for_validation'),
                'stop_trigger':raw.get('stop_trigger'),'stop_result':raw.get('stop_result'),
                'stop_request_elapsed_s':raw.get('stop_request_elapsed_s'),
                'loaded_mplib_modules':data.get('loaded_mplib_modules'),
                'transport_audit_calls':len(audits),'transport_samples':sum(row['samples'] for row in audits),
                'transport_all_world_links_checked':bool(audits) and all(row['world_exempt_links']==[] for row in audits),
                'transport_payload_sphere_counts':sorted({row['payload_spheres'] for row in audits}),
                'read_only_diagnostic_statuses':dict(Counter(row['status'] for row in diagnostics)),
                'diagnostics_keep_table_payload':bool(diagnostics) and all(
                    'virtual_table_plane' in row['world_obstacle_names'] and row['attached_sphere_summary']['active']
                    and row['attached_sphere_summary']['count']>0 for row in diagnostics),
                'policy_rejections':[{k:row.get(k) for k in ('reason','step_id')}
                    for row in evidence if row.get('event')=='transport_policy_rejected'],
                'unanswered_native_input':bool(raw.get('unanswered_native_input'))})
    for path in sorted(root.glob('browser*/report.json')):
        data=json.loads(path.read_text())
        report['browser'].append({'run':path.parent.name,'report_sha256':sha(path),'report':data})
    report['checks']={}
    for name in ('three_scene_tests.log','full_tests.log','compileall.log','snapshot_verify.log'):
        path=root/name
        if path.is_file():
            report['checks'][name]={'sha256':sha(path),
                'last_line':path.read_text().splitlines()[-1] if path.stat().st_size and name.endswith('tests.log') else None}
    source_names=subprocess.check_output(['git','diff','--name-only'],cwd=ROOT,text=True).splitlines()
    source_names+=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd=ROOT,text=True).splitlines()
    report['source_sha256']={name:sha(ROOT/name) for name in sorted(set(source_names))
        if name.endswith(('.py','.js')) and (ROOT/name).is_file()}
    report['notes']=[
        'Keep every failed/repeated native run; v2 worker normal return was a known false completion and is re-evaluated from original final/cycle markers.',
        'Native main-loop cycles count objects; prefetch episode hook invocations do not.',
        'Jimu full v1 did not record a module census; v2 loaded obsolete MPLib bookkeeping. Only later guarded runs can qualify as cuRobo-only.',
        'GPU transport audit covers returned candidate paths, not physical execution or independently observed attachment.',
        'PushT and RRTrack evidence was not rerun here; see three_scene_nomotion_followup_20260907_summary.json.',
        'No push or upload is performed by this command.',
    ]
    atomic_json(args.output,report)
    print(json.dumps({key:len(report[key]) for key in ('pickplace','native_workers','browser')}))


if __name__=='__main__':main()
