#!/usr/bin/env python3
"""Aggregate local validation evidence; no images, native logs or trajectories exported."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from tools.run_pusht_gpu_validation import validation_passed


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.input.resolve()
    report=dict(schema='rm75.three_scene_nomotion/v1',
        tested_base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        hardware_connected=False,execute_real=False,verified_physical_success=None,
        raw_artifacts_local_only=True,pusht=[],pickplace=[],diagnostics=[])
    for path in sorted(root.glob('*/result.json')):
        data=json.loads(path.read_text());name=path.parent.name
        log=root.parent/(root.name+'_'+name+'.log')
        common=dict(run=name,result_sha256=digest(path),
                    log_sha256=digest(log) if log.exists() else None)
        if data.get('gpu_backend')=='curobo2':
            stages=[]
            for stage in data.get('stages',[]):
                stages.append({**{k:v for k,v in stage.items() if k not in ('positions','times')},
                    'timed_samples':len(stage['times']),'duration_s':stage['times'][-1]})
            events=[e for e in data['events'] if e['event']=='push_stage_audited']
            ik=[]
            for event in data['events']:
                if event['event']!='push_cartesian_ik_planned':continue
                rows=event['waypoints']
                ik.append(dict(stage=event['stage'],waypoints=len(rows),
                    min_successful_ik_variants=min(r['successful_ik_variants'] for r in rows),
                    max_successful_ik_variants=max(r['successful_ik_variants'] for r in rows),
                    rejected_edges=sum(len(r['rejections']) for r in rows)))
            report['pusht'].append(dict(**common,case=data['case'],fixture=data['fixture'],
                scenario=data['scenario'],complete_chain=data['complete_chain'],error=data.get('error'),
                validation_success_recorded=data.get('validation_success'),
                dynamics_verified_from_saved_metrics=validation_passed(data),
                expected_blocked_case=data['case']=='orthogonal_neighbor_blocked',
                speed_mps=data['config']['speed_mps'],elapsed_compute_s=data['elapsed_s'],
                trajectory_duration_s=sum(s['duration_s'] for s in stages),stages=stages,
                stage_audits=events,cartesian_ik=ik,
                unrelated_obstacle_audit=data.get('unrelated_obstacle_audit'),
                corridor_tolerance_m=data['motion']['corridor_tolerance_m'],
                orientation_tolerance_rad=data['motion']['orientation_tolerance_rad'],
                hardware_profile_qualified=data['hardware_profile_qualified']))
        elif data.get('planner')=='curobo_only':
            raw=log.read_text(errors='replace') if log.exists() else ''
            report['pickplace'].append(dict(**common,**{k:data.get(k) for k in (
                'case','expected_cycles','command_success','native_cycles','native_final_success',
                'error','clearance_failures','clearance_path_audits','strict_clearance_success',
                'loaded_mplib_modules')},
                release_candidate_rejections=[line for line in raw.splitlines()
                    if line.startswith('[release target-local rejected]')]))
        elif name!='pusht_real_qualification':
            report['diagnostics'].append(dict(**common,
                purpose='Original PushT line feasibility/soft-cost diagnostic, not a chain pass',
                error=data.get('error'),
                coarse_ik_candidates=len(data.get('coarse_metrics',{})),
                coarse_ik_success=sum(r['success'] for r in data.get('coarse_metrics',{}).values()),
                native_linear_cost_attempts=[{k:v for k,v in row.items() if k not in
                    ('q','tcp_positions')} for row in data.get('linear_scale_diagnostic',[])]))
    trajectory=next((root/'jimu_rrtrack_depth_v1').glob('*/trajectory.jsonl'))
    rows=[json.loads(line) for line in trajectory.read_text().splitlines()]
    initial=np.asarray(rows[0]['T_cam_obj']);frames=rows[1:]
    accepted=[r for r in frames if r['accepted']]
    poses=np.asarray([r['T_cam_obj'] for r in accepted]);xyz=poses[:,:3,3]
    # The 6.5-mm CAD thickness is local Y: compare its plane normal, not arbitrary yaw.
    normal_deg=np.degrees(np.arccos(np.clip(poses[:,:3,1]@initial[:3,1],-1,1)))
    report['jimu_rrtrack']=dict(input='recorded_real_RGBD_single_orange_plate',
        tracking_frames=len(frames),accepted=len(accepted),events=dict(Counter(r['event'] for r in frames)),
        initialized_depth=rows[0].get('metadata',{}).get('depth_quality'),
        rejected_frames=[dict(frame_index=r['frame_index'],event=r['event'],
            memory_updates=r['memory_updates'],metadata=r['metadata']) for r in frames if not r['accepted']],
        recoveries=[dict(frame_index=r['frame_index'],source=r['recovery_source'],metadata=r['metadata'])
                    for r in frames if r['event']=='recovered'],
        accepted_position_std_mm=(xyz.std(axis=0)*1000).tolist(),
        accepted_position_range_mm=(np.ptp(xyz,axis=0)*1000).tolist(),
        max_displacement_from_initial_mm=float(np.max(np.linalg.norm(xyz-initial[:3,3],axis=1))*1000),
        plane_normal_change_deg=[float(normal_deg.min()),float(normal_deg.max())],
        absolute_pose_accuracy='NOT_MEASURED',assembly_accuracy_qualified=False,
        physical_occlusion_qualification='NOT_RUN',twelve_real_instances='NOT_RUN',
        trajectory_sha256=digest(trajectory),
        run_config_sha256=digest(trajectory.parent/'run_config.json'),
        log_sha256=digest(root.parent/(root.name+'_jimu_rrtrack_depth_v1.log')))
    report['cpu_checks']={}
    for name in ('three_scene_tests','full_tests','compileall','snapshot_verify'):
        path=root/(name+'.log');lines=path.read_text().strip().splitlines()
        report['cpu_checks'][name]=dict(sha256=digest(path),
            summary=lines[-1] if lines and name!='snapshot_verify' else '')
    snapshot=json.loads((root/'snapshot_verify.log').read_text())
    report['snapshot']={k:snapshot.get(k) for k in
        ('files','source_commit','dependency_completeness_verified','runtime_gpu_verified')}
    report['real_profile_qualification']=json.loads((root/'pusht_real_qualification/result.json').read_text())
    report['totals']=dict(pusht_chain_runs=len(report['pusht']),
        pusht_complete_chains=sum(r['complete_chain'] and not r['error'] for r in report['pusht']),
        pusht_expected_blocked_cases=sum(r['expected_blocked_case'] for r in report['pusht']),
        pusht_actual_obstacle_audit_rejections=sum((r.get('unrelated_obstacle_audit') or {}).get('rejected') is True
                                                 for r in report['pusht']),
        pickplace_runs=len(report['pickplace']),
        pickplace_native_success=sum(r['command_success'] for r in report['pickplace']),
        pickplace_all_clearances_pass=sum(r['strict_clearance_success'] for r in report['pickplace']),
        diagnostic_runs=len(report['diagnostics']))
    source_names=subprocess.check_output(['git','diff','--name-only'],cwd=ROOT,text=True).splitlines()
    source_names+=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd=ROOT,text=True).splitlines()
    report['source_sha256']={name:digest(ROOT/name) for name in sorted(set(source_names))
        if name.endswith('.py') and (ROOT/name).is_file()}
    report['notes']=[
        'All runs, including failed and repeated runs, retained; prior-round 12 GPU failures remain in their historical report.',
        'Authored low-table simulation parameters are not measurements of the real workcell.',
        'Fixed-tool rotated case failure is not replaced by the separate yaw-matched-tool positive fixture.',
        'Early successful GPU runs lack per-point speed metrics; missing evidence is not retroactively invented.',
        'PickPlace original transport masks are not newly relaxed and are not a strict full-chain transport qualification.',
        'Jimu low-depth loss/recovery is real recorded-frame evidence, not physical occlusion or assembly accuracy.',
        'Raw native logs, RGBD, joint trajectories and existing unpushed commits are not uploaded by this summarizer.',
    ]
    atomic_json(args.output,report)
    print(json.dumps(report['totals']))


if __name__=='__main__':main()
