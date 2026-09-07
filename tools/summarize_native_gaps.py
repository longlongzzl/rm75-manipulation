#!/usr/bin/env python3
"""Export compact native validation metrics; raw worlds, q, images stay local."""
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


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def negative_pairs(detail):
    """Whitelist only link/obstacle/clearance; never export raw geometry/joints."""
    contacts=detail.get('robot_world_obstacle_contacts',[])
    if not isinstance(contacts,list):return []
    return [{key:row[key] for key in ('robot_link','obstacle','clearance_m')}
            for row in contacts if isinstance(row,dict) and row.get('clearance_m',0)<0
            and all(key in row for key in ('robot_link','obstacle','clearance_m'))]


def event_summary(path,*,bare=False):
    counts=Counter();near=Counter();pairs={};audits=[];first=None;filters=set()
    with path.open() as stream:
        for line in stream:
            row=json.loads(line)
            if not bare:
                if row.get('kind')!='contact_audit':continue
                row=row['evidence']
            event=row['event'];counts[event]+=1
            if event=='transport_full_world_audit' and row.get('samples',0)>0:
                audits.append({key:row.get(key) for key in
                    ('samples','payload_spheres','world_exempt_links','step_id')})
            if event=='grasp_contact_ik_filter_enter':filters.update(row.get('links',[]))
            if event=='jimu_near_ik_collision_checked':
                near['accepted' if row['accepted'] else 'rejected']+=1
                for contact in negative_pairs(row.get('diagnostic',{})):
                    key=(contact['robot_link'],contact['obstacle'])
                    record=pairs.setdefault(key,{**contact,'observations':0})
                    record['observations']+=1
                    record['clearance_m']=min(record['clearance_m'],contact['clearance_m'])
            if event=='jimu_read_only_collision_diagnostic' and first is None:
                first={'status':row.get('status'),'candidate_label':row.get('candidate_label'),
                       'recorded_q_role':row.get('diagnosed_q_role'),
                       'geometry_detail_recorded':row.get('geometry_detail_recorded'),
                       'negative_pairs':negative_pairs(row)}
    return {'event_counts':dict(counts),'near_ik_promotions':dict(near),
        'near_ik_negative_pairs':sorted(pairs.values(),key=lambda row:row['clearance_m']),
        'first_read_only_diagnostic':first,'grasp_contact_ik_links':sorted(filters),
        'transport_audits':audits,'transport_samples':sum(row['samples'] for row in audits),
        'transport_all_world_links_checked':bool(audits) and all(row['world_exempt_links']==[] for row in audits)}


def pusht_summary(raw):
    fields=('case','fixture','scenario','gpu_backend','hardware_connected','execute_real',
            'hardware_profile_qualified','complete_chain','validation_success','elapsed_s')
    row={key:raw.get(key) for key in fields}
    row['error_type']=str(raw.get('error','')).split(':',1)[0]
    row['unrelated_obstacle_rejected']=raw.get('unrelated_obstacle_audit',{}).get('rejected')
    row['execution_gate_audits']=[{key:item.get(key) for key in
        ('case','passed','rejected','plan','observe','execute','error_type',
         'injected_observation','actual_camera_observation','reused_gpu_prepared_chain')}
        for item in raw.get('execution_gate_audits',[])]
    row['audited_stages_including_partial']=[{key:item.get(key) for key in
        ('stage','samples','duration_s','max_corridor_error_m','max_orientation_error_rad')}
        for item in raw.get('events',[]) if item.get('event')=='push_stage_audited']
    row['stages']=[{**{key:stage.get(key) for key in ('stage','max_tcp_speed_mps',
        'max_joint_speed_rad_s','max_joint_accel_rad_s2')},'samples':len(stage.get('times',[])),
        'duration_s':stage['times'][-1]-stage['times'][0] if stage.get('times') else None}
        for stage in raw.get('stages',[])]
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.input.resolve()
    report={'schema':'rm75.native_gaps/v1','execute_real':False,'hardware_connected':False,
        'camera_captured':False,'physical_success':None,'raw_artifacts_local_only':True,
        'tested_base_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'runs':[],'checks':{},'prior_pusht_evidence':'three_scene_nomotion_followup_20260907_summary.json'}
    for path in sorted(root.glob('*/result.json')):
        raw=json.loads(path.read_text());row={'run':path.parent.name,'result_sha256':sha(path)}
        if raw.get('job_id'):
            fields=('task','completed','command_completed','timeout','elapsed_s','stopped_for_validation','expected_cycles','native_cycles',
                    'native_completed_cycles','native_final_success','native_full_chain_passed','clearance_failures')
            row.update({key:raw.get(key) for key in fields})
            row['original_task_bundle_sha256']=raw.get('original_task_bundle_sha256')
            row['original_task_bundle_read_only']=raw.get('original_task_bundle_read_only',False)
            row['original_task_bundle_unchanged']=raw.get('original_task_bundle_unchanged')
            result=raw.get('job',{}).get('result',{})
            row.update(worker_status=raw.get('job',{}).get('status'),
                fixed_input_sha256=raw.get('fixed_input_sha256'),loaded_mplib_modules=result.get('loaded_mplib_modules'),
                clearance_path_audits=result.get('clearance_path_audits'),
                fixed_scene_format=result.get('fixed_scene_format'),native_entrypoint=result.get('native_entrypoint'))
            row['clearance_selection_audits']=result.get('clearance_selection_audits')
            row['independent_clearance_execution_audit_observed']=result.get('independent_clearance_execution_audit_observed')
            job=ROOT/'runtime_data/workcell/jobs'/raw['job_id']
            row.update(event_summary(job/'events.jsonl'))
            row['events_sha256']=sha(job/'events.jsonl');row['stdout_sha256']=sha(job/'stdout.log')
        elif raw.get('gpu_backend')=='curobo2':row.update(pusht_summary(raw))
        elif raw.get('status')=='qualification_incomplete':
            for key in ('status','execute_real','hardware_connected','gpu_chain_planned','missing_run_inputs'):
                row[key]=raw.get(key)
            qualification=raw.get('qualification',{})
            row['qualification']={key:qualification.get(key) for key in
                ('planning_inputs_complete','integration_qualified','hardware_reviewed','motion_authorized')}
            row['unqualified_fields']=[item.get('field') for item in qualification.get('errors',[])]
        elif 'all_objects_equivalent' in raw:
            for key in ('all_objects_equivalent','source_sha256','calibration_sha256','native_mapping_flags','objects'):
                row[key]=raw.get(key)
            row['fixture_emitted']=(path.parent/'fixed_sam6d_schema.json').is_file()
            row['perception_inference_verified']=False
        elif raw.get('planner')=='curobo_only':
            for key in ('case','scene','strict','elapsed_s','expected_cycles','command_success','native_cycles','native_completed_cycles',
                        'native_final_success','clearance_failures','strict_clearance_success',
                        'clearance_path_audits','clearance_selection_audits','transport_path_audits','loaded_mplib_modules',
                        'independent_clearance_execution_audit_observed'):
                row[key]=raw.get(key)
            events=path.parent/'contact.jsonl'
            if not events.is_file():events=path.parent/'transport.jsonl'
            if events.is_file():row.update(event_summary(events,bare=True));row['events_sha256']=sha(events)
            row['policy_rejections']=[{key:item.get(key) for key in ('reason','step_id')}
                for item in raw.get('transport_policy_rejections',[])]
            row['error_type']=str(raw.get('error','')).split(':',1)[0]
            log=root/(path.parent.name+'.log')
            if log.is_file():row['stdout_sha256']=sha(log)
        else:raise ValueError('Unrecognized local validation result: '+path.parent.name)
        report['runs'].append(row)
    for name in ('three_scene_tests.log','full_tests.log','compileall.log','snapshot_verify.log'):
        path=root/name
        if path.is_file():report['checks'][name]={'sha256':sha(path),
            'last_line':path.read_text().splitlines()[-1] if name.endswith('tests.log') and path.stat().st_size else None}
    paths=subprocess.check_output(['git','diff','--name-only'],cwd=ROOT,text=True).splitlines()
    paths+=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd=ROOT,text=True).splitlines()
    report['source_sha256']={name:sha(ROOT/name) for name in sorted(set(paths)) if name.endswith('.py')}
    report['notes']=[
        'Bounded diagnostic cancellations are not full-chain successes.',
        'Original failed/retried cycles remain in the denominator; physical task success is not inferred.',
        'Near-IK residual acceptance retains original thresholds and now requires current collision validity.',
        'Approved finger world-contact scope applies only to the original grasp-contact IK batch, never transport.',
        'Camera-schema roundtrip failure is retained; no approximate fixture or fresh inference is claimed.',
        'Any PushT runs listed here use real GPU and authored simulation scenes; prior failures remain in the linked earlier summary.',
        'PushT post-plan observation/drift gate rows use injected data and reuse the exact GPU-prepared chain; they are not live camera evidence.',
        'Negative contact pairs are all-link geometric diagnostics, not claims that every listed pair was unmasked in the failed query.',
        'No push or upload is performed by this tool.',
    ]
    atomic_json(args.output,report)
    print(json.dumps({'runs':len(report['runs']),'output':str(args.output)}))


if __name__=='__main__':main()
