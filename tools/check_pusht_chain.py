#!/usr/bin/env python3
"""GPU complete short-push chain from confirmed inputs; never connects an arm."""
import argparse
import json
from pathlib import Path
import sys
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import read_json,atomic_json
from rm75_app.workcell.events import StopToken,EventLog
from rm75_app.workcell.transforms import vector
from rm75_app.pusht.qualification import planning_qualification


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--observation',type=Path,help='Fresh confirmed base-frame tracker observation JSON')
    parser.add_argument('--joints',type=Path,help='Read-only JSON array of seven joint positions in radians')
    parser.add_argument('--goal',type=float,nargs=3,help='Confirmed target x y yaw in base_link')
    args=parser.parse_args();profile=read_json(args.profile)
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    report={'execute_real':False,'hardware_connected':False,'verified_task_success':None,
        'qualification':planning_qualification(profile),'gpu_chain_planned':False}
    missing=[name for name in ('observation','joints','goal') if getattr(args,name) is None]
    if missing or not report['qualification']['planning_inputs_complete']:
        report.update(status='qualification_incomplete',missing_run_inputs=missing)
        atomic_json(output/'result.json',report);print(json.dumps(report));return 2
    backend=None
    try:
        from rm75_app.pusht.model import Config,choose_push
        from rm75_app.pusht.observation import Observation
        from rm75_app.pusht.motion import CuroboPushExecutor
        from rm75_app.planning.backends.curobo2 import Curobo2Backend,Curobo2BackendConfig
        section=profile['pusht'];config=Config.from_dict(section['model'])
        observation=Observation.from_dict(read_json(args.observation))
        observation.validate(max_age_s=config.max_observation_age_s,real=True)
        joints=vector(read_json(args.joints),7,'joint_positions_rad')
        push,prediction=choose_push(observation.pose,args.goal,config)
        class ReadOnlyArm:
            hz=profile['hardware']['stream_hz']
            def read_joints(self):return joints.copy()
            def execute(self,*args,**kwargs):raise AssertionError('No-motion tool cannot execute')
        options=dict(section.get('planner',{}))
        for key in ('robot_config','curobo_root'):
            if key in options:options[key]=Path(options[key]).expanduser().resolve()
        backend=Curobo2Backend(Curobo2BackendConfig(**options))
        executor=CuroboPushExecutor(backend,ReadOnlyArm(),config,section['motion'],
            StopToken(output/'STOP'),EventLog(output),None)
        planned=executor.plan_push(push,observation)
        report.update(status='gpu_chain_planned_no_motion',gpu_chain_planned=True,
            push=push.as_dict(),prediction=prediction,observation=observation.as_dict(),
            stages=[{'stage':stage,'positions':path.tolist(),'times':times.tolist()}
                    for stage,path,times in planned.stages])
    except Exception as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
    finally:
        if backend is not None:backend.__exit__(None,None,None)
        atomic_json(output/'result.json',report)
    print(json.dumps(report))
    return 0 if report['gpu_chain_planned'] else 42


if __name__=='__main__':raise SystemExit(main())
