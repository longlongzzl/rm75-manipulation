#!/usr/bin/env python3
"""Frozen physics cases through the real workcell service/worker, never hardware."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.service import WorkcellService


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=('tool_only_physics','full_arm_physics'),required=True)
    parser.add_argument('--case',choices=('translation','rotation','mixed','neighbor','infeasible','cancel'),required=True)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--timeout-s',type=float,default=900.)
    parser.add_argument('--maximum-push-length-m',type=float,default=.05)
    parser.add_argument('--response-calibration',type=Path,default=None)
    parser.add_argument('--gravity-compensation',choices=('none','original-agent'),default='none')
    parser.add_argument('--gripper-feedback-geometry',action='store_true',
        help='Independent full-arm trial: pose unchanged collision spheres from actual simulated jaw joints')
    parser.add_argument('--closed-gripper-joint-position',type=float,default=.6,
        help='Explicit original URDF coupled-jaw target; .91 is the upper joint limit')
    parser.add_argument('--ik-position-tolerance-m',type=float,default=None,
        help='Independent stricter IK convergence trial; original path corridor is unchanged')
    parser.add_argument('--check-gripper-internal-collisions',action='store_true',
        help='Reproduce the former internal-gripper self-collision check; PushT normally ignores these pairs')
    args=parser.parse_args();output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    if args.gripper_feedback_geometry and args.backend!='full_arm_physics':
        parser.error('Actual articulated jaw feedback requires full-arm physics')
    source=ROOT/'runtime_data/three_scene/pickplace_pusht_followup_20260908/gpu_baseline/input.json'
    frozen=json.loads(source.read_text());motion=copy.deepcopy(frozen['motion'])
    motion['retreat_clearance_m']=.05
    initial=[.35,-.18,0.];goal=[.38,-.18,0.]
    if args.case=='rotation':goal=[.35,-.18,.15]
    if args.case=='mixed':goal=[.38,-.17,.15]
    if args.case in ('neighbor','infeasible'):
        motion['static_collision_objects'].append(dict(name='frozen_neighbor',kind='cuboid',
            position=[.46,-.25,.01] if args.case=='neighbor' else [.25,-.16,.06],
            quaternion_wxyz=[1,0,0,0],dimensions=[.02,.02,.04] if args.case=='neighbor' else [.08,.08,.08]))
    profile=json.loads((ROOT/'examples/workcell/machine.example.json').read_text())
    profile['pusht']['model']={**frozen['config'],'maximum_push_length_m':args.maximum_push_length_m,
                               'horizon':3,'beam_width':12}
    if args.response_calibration is not None:
        calibration=json.loads(args.response_calibration.read_text())
        profile['pusht']['model']['response_fits']=calibration['response_fits']
    profile['pusht']['physics']=dict(enabled_backends=['tool_only_physics','full_arm_physics'],
        simulation_python='/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python',
        planner_python='/home/zhangzhao/anaconda3/envs/curobo2/bin/python',motion=motion,initial_pose=initial,
        gravity_compensation=args.gravity_compensation,gripper_feedback_geometry=args.gripper_feedback_geometry)
    if not 0<=args.closed_gripper_joint_position<=.91:raise ValueError('Gripper target exceeds original URDF limits')
    profile['pusht']['physics']['planner']={'gripper_collision_closed_joint_position':args.closed_gripper_joint_position,
        'ignore_gripper_internal_self_collision':not args.check_gripper_internal_collisions}
    if args.ik_position_tolerance_m is not None:
        if not .0001<=args.ik_position_tolerance_m<=.005:raise ValueError('Only stricter IK tolerance is allowed')
        profile['pusht']['physics']['planner']['position_tolerance']=args.ik_position_tolerance_m
    assert profile['hardware']['hardware_reviewed'] is False
    atomic_json(output/'machine.json',profile)
    spec=dict(task='pusht',mode='sim',parameters=dict(initial_pose=initial,goal_pose=goal,
        speed_mps=.015,max_steps=12,simulation_backend=args.backend))
    atomic_json(output/'request.json',spec)
    report=dict(case=args.case,backend=args.backend,execute_real=False,hardware_connected=False,
        push_search_model=profile['pusht']['model'],
        response_calibration=str(args.response_calibration) if args.response_calibration else None,
        ik_position_tolerance_m=args.ik_position_tolerance_m or .005,
        gravity_compensation=args.gravity_compensation,
        gripper_feedback_geometry=args.gripper_feedback_geometry,retreat_collision_checks=False,
        closed_gripper_joint_position=args.closed_gripper_joint_position,
        ignore_gripper_internal_self_collision=not args.check_gripper_internal_collisions,
        frozen_base_input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),request=spec,
        expected='cancel' if args.case=='cancel' else 'reject' if args.case=='infeasible' else 'goal',
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [ROOT/'rm75_app/pusht/physics.py',ROOT/'rm75_app/simulation/pusht_loop_env.py',
                      ROOT/'tools/pusht_physics_planner.py',ROOT/'rm75_app/pusht/controller.py',
                      ROOT/'rm75_app/pusht/motion.py',ROOT/'rm75_app/pusht/closed_gripper.py',
                      ROOT/'rm75_app/pusht/model.py',ROOT/'rm75_app/pusht/batch_search.py',
                      ROOT/'rm75_app/pusht/response.py',
                      ROOT/'rm75_app/planning/backends/curobo2.py']})
    service=WorkcellService(ROOT,output/'machine.json',allow_real=False);tick=time.monotonic();job=None
    try:
        job=service.submit(spec)['job_id'];report['job_id']=job;print(json.dumps(dict(job_id=job)),flush=True)
        cancelled=False
        while service.active:
            state=service.job(job)
            if (args.case=='cancel' and not cancelled and
                    any(row.get('kind')=='physics_replan' and row.get('plan',{}).get('complete') for row in state.get('events',[]))):
                cancel_time=time.monotonic();report['cancel_result']=service.cancel(job)
                report['cancel_wall_s']=time.monotonic()-cancel_time;cancelled=True
            if time.monotonic()-tick>args.timeout_s:
                report['timeout']=True;service.cancel(job);break
            time.sleep(.1)
        service.close();report['job']=service.job(job)
    except BaseException as exc:report['error']=f'{type(exc).__name__}: {exc}'
    finally:
        service.close();report['elapsed_s']=time.monotonic()-tick;atomic_json(output/'result.json',report)
    result=report.get('job',{}).get('result',{})
    print(json.dumps(dict(case=args.case,backend=args.backend,status=result.get('status'),
        verification=result.get('verification'),error=result.get('error'),elapsed_s=report['elapsed_s'])))


if __name__=='__main__':main()
