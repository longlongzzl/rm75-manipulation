#!/usr/bin/env python3
"""Export compact native validation metrics; raw worlds, q, images stay local."""
import argparse
from collections import Counter
import hashlib
import json
import math
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


def lift_ik_summary(row):
    """Whitelist failed-query metrics, excluding raw q, poses, worlds and paths."""
    def configuration(raw):
        return {'valid':raw.get('valid'),'native_status':raw.get('native_status'),
            'box_contacts':[{key:item.get(key) for key in ('link','obstacle','overlap_m','enabled')}
                            for item in raw.get('box_contacts',[])],
            'self_link_pairs':[{key:item.get(key) for key in ('link_a','link_b','overlap','count')}
                               for item in raw.get('self_collision',{}).get('link_pairs',[])],
            'non_box_objects_not_analytically_audited':raw.get('non_box_objects_not_analytically_audited')}
    result={key:row.get(key) for key in ('source','step_id','diagnostic_only','execution_guard',
        'native_success','native_status','requested_num_seeds','diagnostic_complete','state_unchanged',
        'attached','payload_spheres','collision_model_sha256')}
    result['diagnostic_error_type']=str(row.get('diagnostic_error','')).split(':',1)[0]
    result['start']=configuration(row.get('start',{}))
    result['returned_rows']=[{**{key:item.get(key) for key in
        ('index','native_success','finite','position_error_m','rotation_error_native')},
        'configuration':configuration(item.get('configuration',{}))} for item in row.get('returned_rows',[])]
    nominal=row.get('nominal_goal_payload_base')
    result['nominal_goal_payload_base']=None if nominal is None else {
        **{key:nominal.get(key) for key in ('payload_spheres','base_spheres','base_comparison_max_delta_m',
            'geometric_necessary_condition_only','physical_geometry_qualified')},
        'contacts':[{key:item.get(key) for key in ('link_a','link_b','overlap_m','pair_ignored')}
                    for item in nominal.get('contacts',[])]}
    return result


def roof_ik_summary(row):
    """Keep same-seed scalar errors and contact pairs, never raw q/poses/worlds."""
    result={key:row.get(key) for key in ('source','phase','step_id','prefetch',
        'diagnostic_only','execution_guard','requested_num_seeds','goal_count',
        'native_success_count','original_near_position_threshold','original_near_rotation_threshold',
        'diagnostic_complete','state_unchanged','attached','payload_spheres','collision_model_sha256')}
    result['diagnostic_error_type']=str(row.get('diagnostic_error','')).split(':',1)[0]
    summaries=[];mismatches=0;unsafe_threshold_pairs=0
    def finite(value):return isinstance(value,(int,float)) and math.isfinite(value)
    for goal in row.get('goals',[]):
        seeds=goal['seeds'];index=goal['legacy_nearest_seed_index'];seed=seeds[index]
        debug=(goal.get('debug_position_error_m'),goal.get('debug_rotation_error_native'))
        errors=(seed.get('position_error_m'),seed.get('rotation_error_native'))
        available=all(finite(v) for v in (*debug,*errors))
        mismatch=available and any(not math.isclose(a,b,rel_tol=1e-6,abs_tol=1e-9)
                                   for a,b in zip(debug,errors))
        thresholds=(row.get('original_near_position_threshold'),row.get('original_near_rotation_threshold'))
        unsafe=bool(available and all(finite(v) for v in thresholds)
            and all(a<=t for a,t in zip(debug,thresholds))
            and any(a>t for a,t in zip(errors,thresholds)))
        mismatches+=int(mismatch);unsafe_threshold_pairs+=int(unsafe)
        summaries.append(dict(goal_index=goal['goal_index'],native_success=goal['native_success'],
            native_status=goal['native_status'],returned_seed_count=len(seeds),
            raw_success_rows=sum(s['native_success'] is True for s in seeds),
            legacy_nearest_seed_index=index,nearest_seed_finite=seed['finite'],
            nearest_position_error_m=errors[0],nearest_rotation_error_native=errors[1],
            debug_position_error_m=debug[0],debug_rotation_error_native=debug[1],
            comparison_available=available,debug_nearest_error_mismatch=bool(mismatch),
            debug_passes_but_nearest_fails_original_thresholds=unsafe))
    result['captured_goal_count']=len(summaries)
    result['error_comparison_available_count']=sum(s['comparison_available'] for s in summaries)
    result['returned_seed_count_histogram']=dict(Counter(str(s['returned_seed_count']) for s in summaries))
    result['raw_success_rows']=sum(s['raw_success_rows'] for s in summaries)
    result['debug_nearest_error_mismatch_count']=mismatches
    result['debug_passes_but_nearest_fails_original_thresholds_count']=unsafe_threshold_pairs
    examples=[];selected={d['goal_index'] for d in row.get('configuration_details',[])}
    if summaries:selected.add(summaries[0]['goal_index'])
    for item in summaries:
        if item['goal_index'] in selected or item['debug_nearest_error_mismatch']:
            examples.append(item)
        if len(examples)>=8:break
    result['goals']=examples
    result['goal_examples_only']=True
    errors=[s['nearest_position_error_m'] for s in summaries
            if not s['native_success'] and finite(s['nearest_position_error_m'])]
    result['failed_nearest_position_error_m']={'count':len(errors),
        'min':min(errors) if errors else None,'max':max(errors) if errors else None}
    result['configuration_details']=[]
    for raw in row.get('configuration_details',[]):
        detail={key:raw.get(key) for key in ('goal_index','seed_index','valid','native_status',
            'independently_measured_position_error_m','non_box_objects_not_analytically_audited')}
        detail['box_contacts']=[{key:item.get(key) for key in ('link','obstacle','overlap_m','enabled')}
                                for item in raw.get('box_contacts',[])]
        detail['self_link_pairs']=[{key:item.get(key) for key in ('link_a','link_b','overlap','count')}
                                   for item in raw.get('self_collision',{}).get('link_pairs',[])]
        detail['self_diagnostic_is_native_conditional']=True
        result['configuration_details'].append(detail)
    nominal=row.get('nominal_gripper_base_boxes')
    result['nominal_gripper_base_boxes']=None
    if nominal is not None:
        aggregate={}
        for goal in nominal['goals']:
            for contact in goal['box_contacts']:
                key=(contact['link'],contact['obstacle'],contact['enabled'])
                item=aggregate.setdefault(key,{'goal_indices':set(),'max_overlap_m':0.})
                item['goal_indices'].add(goal['goal_index'])
                item['max_overlap_m']=max(item['max_overlap_m'],contact['overlap_m'])
        result['nominal_gripper_base_boxes']={**{key:nominal.get(key) for key in
            ('link','sphere_count','rigid_comparison_max_delta_m','geometric_necessary_condition_only',
             'physical_geometry_qualified','non_box_objects_not_analytically_audited')},
            'goal_count':len(nominal['goals']),
            'goals_with_enabled_box_overlap':sum(g['enabled_box_overlap'] for g in nominal['goals']),
            'contact_pairs':[dict(link=key[0],obstacle=key[1],enabled=key[2],
                goal_count=len(value['goal_indices']),max_overlap_m=value['max_overlap_m'])
                for key,value in sorted(aggregate.items())]}
        goals=nominal['goals'];count=len(goals)
        if (row.get('phase')=='paired_place' and count and count%2==0
                and count==len(row.get('goals',[]))
                and [g['goal_index'] for g in goals]==list(range(count))):
            # Verified native call site: q_grasps+q_grasps, hover_poses+release_poses.
            half=count//2;hover=[g['enabled_box_overlap'] for g in goals[:half]]
            release=[g['enabled_box_overlap'] for g in goals[half:]]
            result['nominal_gripper_base_boxes']['paired_candidates']={
                'count':half,'native_goal_order':'hover_then_release',
                'hover_with_enabled_box_overlap':sum(hover),
                'release_with_enabled_box_overlap':sum(release),
                'either_goal_with_enabled_box_overlap':sum(h or r for h,r in zip(hover,release))}
    return result


def event_summary(path,*,bare=False):
    counts=Counter();near=Counter();pairs={};audits=[];first=None;first_return=None;release_observations=[];release_audits=[];filters=set()
    return_probes=[];roof_ik=[];paired_filters=set()
    with path.open() as stream:
        for line in stream:
            row=json.loads(line)
            if not bare:
                if row.get('kind')!='contact_audit':continue
                row=row['evidence']
            event=row['event'];counts[event]+=1
            if event=='jimu_roof_ik_batch_diagnostic':roof_ik.append(roof_ik_summary(row))
            if event=='contact_world_filter_enter' and 'paired_relation_ik' in row.get('step_id',''):
                paired_filters.update(row.get('links',[]))
            if event=='jimu_independent_return_gate_probe':
                probe={key:row.get(key) for key in ('passed','state_unchanged','error_type','negative_native_status',
                    'negative_state_queries','reused_gpu_planned_return','injected_sim_start','actual_execute_calls',
                    'fresh_planner_chain','physical_success')}
                probe['cases']=[{**{key:item.get(key) for key in ('case','label','passed','rejected','expected_rejected',
                    'non_motion_sink_calls','actual_execute_calls','error_type')},
                    'execution_audit':{key:(item.get('execution_audit') or {}).get(key) for key in
                        ('stage_kind','execution_entry','execution_guard','passed','state_unchanged','return_samples',
                         'clearance_samples','release_contact_samples','permitted_contact_target','permitted_links',
                         'return_world_exempt_links','self_collision_input_modified','world_filter_calls','error_type')}}
                    for item in row.get('cases',[])]
                return_probes.append(probe)
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
            if event=='jimu_return_query_diagnostic' and first_return is None:
                first_return={key:row.get(key) for key in ('step_id','source','native_status',
                    'native_success','diagnostic_complete','state_unchanged','released','attached',
                    'world_constraint_enabled','self_constraint_enabled','disabled_links','disabled_objects')}
                first_return['endpoints']={name:{'valid':detail.get('valid'),'status':detail.get('status'),
                    'geometry_detail_recorded':detail.get('geometry_detail_recorded'),
                    'negative_pairs':negative_pairs(detail)} for name,detail in row.get('endpoints',{}).items()}
            if event=='jimu_release_execution_observation':
                observation={key:row.get(key) for key in
                    ('step_id','source','stage_kind','diagnostic_only','execution_guard','diagnostic_complete','state_unchanged',
                     'released','attached','table_present','max_gripper_model_error_rad','path_points','audited_samples',
                     'invalid_samples','native_all_valid','disabled_links','disabled_objects',
                     'world_constraint_enabled','self_constraint_enabled','model_sync_requested')}
                observation['first_invalid']=[{**{key:item.get(key) for key in
                    ('index','status','geometry_detail_recorded')},'negative_pairs':negative_pairs(item)}
                    for item in row.get('first_invalid',[])]
                release_observations.append(observation)
            if event=='jimu_release_execution_audit':
                release_audits.append({key:row.get(key) for key in
                    ('step_id','source','stage_kind','execution_entry','entry_connector_audited','execution_guard','passed','state_unchanged','clearance_samples',
                     'return_samples','release_contact_samples','permitted_contact_target','permitted_links',
                     'return_world_exempt_links','self_collision_input_modified','world_filter_calls','error_type')})
    return {'event_counts':dict(counts),'near_ik_promotions':dict(near),
        'near_ik_negative_pairs':sorted(pairs.values(),key=lambda row:row['clearance_m']),
        'first_read_only_diagnostic':first,'first_return_query_diagnostic':first_return,
        'release_execution_observations':release_observations,
        'jimu_release_execution_audits':release_audits,
        'independent_return_gate_probes':return_probes,
        'roof_ik_diagnostics':roof_ik,
        'paired_relation_ik_world_exempt_links':sorted(paired_filters),
        'grasp_contact_ik_links':sorted(filters),
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
        if 'roof_ik_audit' in raw:row['roof_ik_audit']=raw['roof_ik_audit']
        if 'validation_success' in raw:row['validation_success']=raw['validation_success']
        if raw.get('job_id'):
            fields=('task','completed','command_completed','timeout','elapsed_s','stopped_for_validation','expected_cycles','native_cycles',
                    'native_completed_cycles','native_final_success','native_full_chain_passed','clearance_failures')
            row.update({key:raw.get(key) for key in fields})
            row['original_task_bundle_sha256']=raw.get('original_task_bundle_sha256')
            row['original_task_bundle_read_only']=raw.get('original_task_bundle_read_only',False)
            row['original_task_bundle_unchanged']=raw.get('original_task_bundle_unchanged')
            row['documented_start_comparison']=raw.get('documented_start_comparison')
            row['documented_start_source_unchanged']=raw.get('documented_start_source_unchanged')
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
            row['lift_ik_diagnostics']=[lift_ik_summary(item) for item in raw.get('lift_ik_diagnostics',[])]
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
