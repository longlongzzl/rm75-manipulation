#!/usr/bin/env python3
"""Bounded original-session simulation probe, not SWM task qualification.

Run exclusively through tools/run_network_isolated.py. No hardware API is used.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import uuid
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.pusht.physics import PhysicsSession
from rm75_app.pusht.physics_replay import TimedStageProgram
from rm75_app.pusht.model import Config,Push
from rm75_app.workcell.events import EventLog,StopToken
from rm75_app.workcell.io import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    profile=json.loads(args.profile.read_text());section=profile['pusht']
    if profile['hardware'].get('hardware_reviewed') is not False:
        raise PermissionError('Probe requires an explicitly unqualified simulation profile')
    spec=dict(mode='sim',task='pusht',parameters=dict(simulation_backend='full_arm_physics',
        initial_pose=section['physics']['initial_pose']))
    events=EventLog(output);stop=StopToken(output/'STOP')
    session=PhysicsSession(spec,section,Config.from_dict(section['model']),stop,events)
    result=dict(scope='original_native_approach_stage_probe',task_success=False,
        swm_scene_audit_verified=False,hardware_connected=False,model_inference='NOT_RUN',
        status='FAILED',resources_closed=False,
        profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest())
    try:
        session.open()
        session.feedback_action_id=uuid.uuid4().hex
        session._prepared_contact_binding=None
        before=session.observe();before_tool=session.measured_tool_feedback()
        q=session.base.read_q();goal=q.copy();goal[0]+=.02
        reference=TimedStageProgram('approach',np.array([q,goal]),np.array([0.,1.]),session.fk)
        push=Push((.3,-.18),(1.,0.),.02,.015)
        planned=session.replan_measured_stage(reference,push,before)
        session.check_stage_start(planned)
        session.base.active_program=planned;session.base.program_started=session.base.physics_time
        session.advance(planned.duration);session.advance(1.)
        after=session.observe(after=before.captured_at)
        after.validate(now=session.clock(),after=before.captured_at,previous=before,
                       max_age_s=session.config.max_observation_age_s)
        after_tool=session.measured_tool_feedback()
        gap=float(np.max(abs(session.base.read_q()-planned.positions[-1])))
        result.update(before=before.as_dict(),after=after.as_dict(),
            before_tool=before_tool,after_tool=after_tool,
            actual_action_id=session.feedback_action_id,planned_samples=len(planned.positions),
            planned_duration_s=planned.duration,final_joint_error_rad=gap,
            original_final_joint_error_limit_rad=.02)
        if gap>.02:raise RuntimeError('Articulated final joint tracking error')
        result['status']='SIMULATED_STAGE_COMPLETED_NOT_TASK_QUALIFIED'
    except BaseException as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
    finally:
        try:session.close();result['resources_closed']=True
        except BaseException as exc:result['close_error']=f'{type(exc).__name__}: {exc}'
        atomic_json(output/'result.json',result)
    print(json.dumps(result,allow_nan=False),flush=True)
    return 0 if result['status']=='SIMULATED_STAGE_COMPLETED_NOT_TASK_QUALIFIED' and result['resources_closed'] else 1


if __name__=='__main__':raise SystemExit(main())
