#!/usr/bin/env python3
"""Fit current PushT response from completed isolated full-arm simulation jobs."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.pusht.model import Config,Push
from rm75_app.pusht.response import ResponseEstimator
from rm75_app.workcell.io import atomic_json


def fit_jobs(jobs,output):
    estimator=None;transitions=[];sources=[];signature=None
    for job in jobs:
        summary=json.loads((job/'physics/summary.json').read_text())
        profile=json.loads((job/'physics/planner_profile.json').read_text())
        if (summary.get('execute_real') is not False or summary.get('hardware_connected') is not False
                or summary.get('backend')!='full_arm_physics'):
            raise ValueError('Only isolated full-arm physics evidence is accepted')
        geometry={k:profile['model'][k] for k in ('bar_width_m','bar_height_m','stem_width_m',
                                                  'stem_height_m','pusher_radius_m')}
        contact_geometry={k:profile['motion'][k] for k in ('tool_frame','tool_quaternion_wxyz',
            'push_tcp_z_m','object_height_m','object_centroid_z_m','pusher_contact_links','static_collision_objects')}
        current_signature=(geometry,contact_geometry,summary['urdf_sha256'],summary['planned_gripper_joint_targets'])
        if signature is not None and current_signature!=signature:
            raise ValueError('Cannot mix different object/tool geometry or gripper targets')
        signature=current_signature
        if estimator is None:estimator=ResponseEstimator(Config.from_dict({**profile['model'],'response_fits':[]}))
        events_path=job/'events.jsonl'
        events=[json.loads(s) for s in events_path.read_text().splitlines()]
        poses={e['step']:e['observation']['pose'] for e in events if e['kind']=='observation'}
        completed={e['step'] for e in events if e['kind']=='push_command_finished'}
        for e in events:
            if e['kind']!='push_planned' or e['step'] not in completed or e['step']+1 not in poses:continue
            step=e['step'];row=estimator.update(poses[step],Push(**e['push']),poses[step+1])
            transitions.append(dict(job=job.name,step=step,before=poses[step],push=e['push'],
                                    after=poses[step+1],feature=row['feature']))
        sources.append(dict(job=job.name,events_sha256=hashlib.sha256(events_path.read_bytes()).hexdigest(),
            planner_profile_sha256=hashlib.sha256((job/'physics/planner_profile.json').read_bytes()).hexdigest()))
    if not transitions:raise ValueError('No completed observed transitions')
    report=dict(execute_real=False,hardware_connected=False,source='completed_full_arm_physics_transitions',
                response_fits=list(estimator.config().response_fits),transitions=transitions,sources=sources,
                calibration_is_real_hardware_qualification=False)
    atomic_json(output,report)
    print(json.dumps(dict(completed_transitions=len(transitions),fitted_contacts=len(report['response_fits']))))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();fit_jobs([p.resolve() for p in args.job],args.output.resolve())


if __name__=='__main__':main()
