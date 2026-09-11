#!/usr/bin/env python3
"""Frozen physics cases through the real workcell service/worker, never hardware."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.service import WorkcellService

# Fixed initial/goal per frozen case name. 'random' runs the seeded table below
# through the same service flow with identical parameters; seeds are fixed in
# code so every run is reproducible, never drawn from an RNG at run time.
CASES={
    'translation':([.35,-.18,0.],[.38,-.18,0.]),
    'rotation':([.35,-.18,0.],[.35,-.18,.15]),
    'mixed':([.35,-.18,0.],[.38,-.17,.15]),
    'neighbor':([.35,-.18,0.],[.38,-.18,0.]),
    'infeasible':([.35,-.18,0.],[.38,-.18,0.]),
    'cancel':([.35,-.18,0.],[.38,-.18,0.]),
}
RANDOM_CASES=[
    ('r1_yaw_recovery',[.34,-.19,.2],[.38,-.18,0.]),
    ('r2_rotated_goal',[.35,-.18,0.],[.37,-.17,.12]),
    ('r3_offset_start',[.33,-.16,-.1],[.38,-.18,0.]),
]


def frozen_profile(args,output_dir,initial,goal,case):
    source=ROOT/'runtime_data/three_scene/pickplace_pusht_followup_20260908/gpu_baseline/input.json'
    frozen=json.loads(source.read_text());motion=copy.deepcopy(frozen['motion'])
    motion['retreat_clearance_m']=.05
    if case in ('neighbor','infeasible'):
        motion['static_collision_objects'].append(dict(name='frozen_neighbor',kind='cuboid',
            position=[.46,-.25,.01] if case=='neighbor' else [.25,-.16,.06],
            quaternion_wxyz=[1,0,0,0],dimensions=[.02,.02,.04] if case=='neighbor' else [.08,.08,.08]))
    profile=json.loads((ROOT/'examples/workcell/machine.example.json').read_text())
    profile['pusht']['model']={**frozen['config'],'maximum_push_length_m':args.maximum_push_length_m,
                               'horizon':3,'beam_width':12}
    if args.response_calibration is not None:
        calibration=json.loads(args.response_calibration.read_text())
        profile['pusht']['model']['response_fits']=calibration['response_fits']
    physics=dict(enabled_backends=['tool_only_physics','full_arm_physics'],
        simulation_python='/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python',
        planner_python='/home/zhangzhao/anaconda3/envs/curobo2/bin/python',motion=motion,initial_pose=initial,
        gravity_compensation=args.gravity_compensation,gripper_feedback_geometry=args.gripper_feedback_geometry)
    profile['pusht']['physics']=physics
    parameters=dict(initial_pose=initial,goal_pose=goal,speed_mps=.015,max_steps=12,
                    simulation_backend=args.backend)
    if args.confirm_window_relocate is not None:
        # Interactive session split: the machine profile carries the boundary
        # control policy, the typed request only enables it. The worker itself
        # writes session_policy.json before building the controller, so no
        # harness step writes worker files behind its back.
        profile['pusht']['session_policy']=dict(position_replan_m=.003,yaw_replan_rad=.04,
                                                poll_s=.1,max_wall_s=0.)
        parameters['run_until_goal']=True
    if not 0<=args.closed_gripper_joint_position<=.91:raise ValueError('Gripper target exceeds original URDF limits')
    physics['planner']={'gripper_collision_closed_joint_position':args.closed_gripper_joint_position,
        'ignore_gripper_internal_self_collision':not args.check_gripper_internal_collisions}
    if args.ik_position_tolerance_m is not None:
        if not .0001<=args.ik_position_tolerance_m<=.005:raise ValueError('Only stricter IK tolerance is allowed')
        physics['planner']['position_tolerance']=args.ik_position_tolerance_m
    if args.disturb_after_pushes is not None:
        from rm75_app.pusht.model import Config,valid_pose
        model=Config.from_dict(profile['pusht']['model'])
        if not valid_pose(args.disturb_pose,model):raise ValueError('Disturbance pose leaves the T workspace')
        physics['disturbances']=[dict(after_pushes=args.disturb_after_pushes,pose=list(args.disturb_pose))]
    if args.success_dwell_s is not None:
        if not .3<=args.success_dwell_s<=5.:raise ValueError('success dwell must stay within .3..5 s')
        profile['pusht']['model']['success_dwell_s']=args.success_dwell_s
    assert profile['hardware']['hardware_reviewed'] is False
    atomic_json(output_dir/'machine.json',profile)
    spec=dict(task='pusht',mode='sim',parameters=parameters)
    atomic_json(output_dir/'request.json',spec)
    report=dict(case=case,backend=args.backend,execute_real=False,hardware_connected=False,
        push_search_model=profile['pusht']['model'],
        response_calibration=str(args.response_calibration) if args.response_calibration else None,
        ik_position_tolerance_m=args.ik_position_tolerance_m or .005,
        gravity_compensation=args.gravity_compensation,
        gripper_feedback_geometry=args.gripper_feedback_geometry,retreat_collision_checks=False,
        closed_gripper_joint_position=args.closed_gripper_joint_position,
        ignore_gripper_internal_self_collision=not args.check_gripper_internal_collisions,
        disturbance_schedule=physics.get('disturbances'),
        session_policy=profile['pusht'].get('session_policy'),
        confirmation_intervention=(list(args.confirm_window_relocate)
            if getattr(args,'confirm_window_relocate',None) is not None else None),
        frozen_base_input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),request=spec,
        expected='cancel' if case=='cancel' else 'reject' if case=='infeasible' else 'goal',
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [ROOT/'rm75_app/pusht/physics.py',ROOT/'rm75_app/simulation/pusht_loop_env.py',
                      ROOT/'tools/pusht_physics_planner.py',ROOT/'rm75_app/pusht/controller.py',
                      ROOT/'rm75_app/pusht/motion.py',ROOT/'rm75_app/pusht/closed_gripper.py',
                      ROOT/'rm75_app/pusht/model.py',ROOT/'rm75_app/pusht/batch_search.py',
                      ROOT/'rm75_app/pusht/response.py',ROOT/'rm75_app/pusht/session_control.py',
                      ROOT/'rm75_app/workcell/iteration_api.py',
                      ROOT/'rm75_app/planning/backends/curobo2.py']})
    return report


def _await_session(api,job_id,predicate,timeout_s=60.):
    deadline=time.monotonic()+timeout_s
    while time.monotonic()<deadline:
        state=api.session(job_id)
        if predicate(state): return state
        time.sleep(.05)
    raise TimeoutError('Session control acknowledgement timeout')


def _confirmation_intervention(api,job_id,pose,tick):
    """Pause, relocate and resume through the real session-control API.

    IterationAPI.control is the exact entry the browser route
    POST /api/workcell/iterate/sessions/<id>/control calls: it re-validates the
    request, requires an acknowledged boundary pause before a SIM relocation and
    writes the numbered session command the worker's SessionControl polls. Only
    the HTTP transport is out of scope here (this process runs under the
    non-Unix-socket sandbox); no harness step writes worker files directly.
    """
    steps=[]
    for action,payload in (('pause',dict(action='pause')),
                           ('relocate',dict(action='relocate',pose=list(pose))),
                           ('resume',dict(action='resume'))):
        if action=='relocate':
            before=_await_session(api,job_id,
                lambda state:state.get('phase')=='paused' and state.get('safe_to_adjust') is True)
        else:
            before=api.session(job_id)
        reply=api.control(job_id,payload)
        after=_await_session(api,job_id,
            lambda state,number=reply['sequence']:state.get('last_command',0)>=number)
        steps.append(dict(action=action,reply=reply,at=time.monotonic()-tick,
            phase_before=before.get('phase'),phase_after=after.get('phase'),
            epoch_after=after.get('epoch'),last_pose=after.get('last_pose')))
    return steps


def run_single(args,output_dir,case_name,initial,goal):
    report=frozen_profile(args,output_dir,initial,goal,case_name)
    # Each case gets its own service so the per-case machine profile (initial
    # pose, disturbances) is the one the worker snapshot sees.
    service=WorkcellService(ROOT,output_dir/'machine.json',allow_real=False)
    tick=time.monotonic();job=None
    try:
        try:
            job=service.submit(dict(task='pusht',mode='sim',parameters=dict(initial_pose=initial,goal_pose=goal,
                speed_mps=.015,max_steps=12,simulation_backend=args.backend,
                **({'run_until_goal':True} if args.confirm_window_relocate is not None else {}))))['job_id']
            report['job_id']=job
            print(json.dumps(dict(job_id=job,case=case_name)),flush=True)
            cancelled=False
            intervened=args.confirm_window_relocate is None
            api=None
            while service.active:
                state=service.job(job)
                # The first goal_confirmation opens the window the intervention
                # must land in, so that event is the trigger.
                if (args.confirm_window_relocate is not None and not intervened and
                        any(row.get('kind')=='goal_confirmation' for row in state.get('events',[]))):
                    if api is None:
                        from rm75_app.workcell.iteration_api import IterationAPI
                        api=IterationAPI(service)
                    steps=_confirmation_intervention(api,job,args.confirm_window_relocate,tick)
                    intervened=True
                    report['confirmation_intervention']=dict(pose=list(args.confirm_window_relocate),
                        goal_confirmed_once=True,api='IterationAPI.control',steps=steps)
                if (case_name=='cancel' and not cancelled and
                        any(row.get('kind')=='physics_replan' and row.get('plan',{}).get('complete') for row in state.get('events',[]))):
                    cancel_time=time.monotonic();report['cancel_result']=service.cancel(job)
                    report['cancel_wall_s']=time.monotonic()-cancel_time;cancelled=True
                if time.monotonic()-tick>args.timeout_s:
                    report['timeout']=True;service.cancel(job);break
                time.sleep(.1)
            report['job']=service.job(job)
        except BaseException as exc:report['error']=f'{type(exc).__name__}: {exc}'
        finally:
            report['elapsed_s']=time.monotonic()-tick;atomic_json(output_dir/'result.json',report)
    finally:
        service.close()
    result=report.get('job',{}).get('result',{})
    print(json.dumps(dict(case=case_name,backend=args.backend,status=result.get('status'),
        verification=result.get('verification'),error=result.get('error'),elapsed_s=report['elapsed_s'])),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=('tool_only_physics','full_arm_physics'),required=True)
    parser.add_argument('--case',choices=tuple(CASES)+('random',),required=True)
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
    parser.add_argument('--disturb-after-pushes',type=int,default=None,
        help='Recovery test: externally move the T to --disturb-pose after this many completed pushes')
    parser.add_argument('--disturb-pose',default=None,
        help='x,y,yaw (meters, radians) disturbance pose; requires --disturb-after-pushes')
    parser.add_argument('--confirm-window-relocate',default=None,
        help='Intervention inside the goal-confirmation window: pause, relocate T to '
             'x,y,yaw, then resume through the real session-command path')
    parser.add_argument('--success-dwell-s',type=float,default=None,
        help='Confirmation pacing only (NOT a success threshold): widen the dwell so '
             'the external intervention can land inside the confirmation window')
    args=parser.parse_args()
    if (args.disturb_after_pushes is None)!=(args.disturb_pose is None):
        parser.error('--disturb-after-pushes and --disturb-pose must be given together')
    if args.confirm_window_relocate is not None:
        try:
            relocate_parts=[float(v) for v in args.confirm_window_relocate.split(',')]
        except ValueError:
            parser.error('--confirm-window-relocate must be x,y,yaw')
        if len(relocate_parts)!=3 or not all(math.isfinite(v) for v in relocate_parts) or abs(relocate_parts[2])>math.pi:
            parser.error('--confirm-window-relocate must be finite x,y and |yaw|<=pi')
        args.confirm_window_relocate=tuple(relocate_parts)
        if args.case!='translation':
            parser.error('The confirmation-window intervention starts from the original translation case')
    if args.disturb_after_pushes is not None:
        if args.disturb_after_pushes<1:parser.error('--disturb-after-pushes must be at least 1')
        try:
            parts=[float(v) for v in args.disturb_pose.split(',')]
        except ValueError:
            parser.error('--disturb-pose must be x,y,yaw')
        if len(parts)!=3 or not all(math.isfinite(v) for v in parts) or abs(parts[2])>math.pi:
            parser.error('--disturb-pose must be finite x,y and |yaw|<=pi')
        args.disturb_pose=tuple(parts)
    if args.gripper_feedback_geometry and args.backend!='full_arm_physics':
        parser.error('Actual articulated jaw feedback requires full-arm physics')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    if args.case!='random':
        initial,goal=CASES[args.case]
        run_single(args,output,args.case,initial,goal)
    else:
        rows=[]
        for name,initial,goal in RANDOM_CASES:
            sub=output/name;sub.mkdir(exist_ok=False)
            report=run_single(args,sub,name,initial,goal)
            rows.append(dict(name=name,initial=initial,goal=goal,
                status=report.get('job',{}).get('result',{}).get('status'),
                verification=report.get('job',{}).get('result',{}).get('verification'),
                error=report.get('error') or report.get('job',{}).get('result',{}).get('error'),
                job_id=report.get('job_id'),elapsed_s=report.get('elapsed_s')))
        atomic_json(output/'random_summary.json',dict(backend=args.backend,
            maximum_push_length_m=args.maximum_push_length_m,
            response_calibration=str(args.response_calibration) if args.response_calibration else None,
            closed_gripper_joint_position=args.closed_gripper_joint_position,
            gravity_compensation=args.gravity_compensation,
            gripper_feedback_geometry=args.gripper_feedback_geometry,
            execute_real=False,hardware_connected=False,cases=rows))
        print(json.dumps(dict(case='random',rows=rows)),flush=True)


if __name__=='__main__':
    main()
