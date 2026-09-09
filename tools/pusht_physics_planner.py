#!/usr/bin/env python3
"""Persistent cuRobo2 planning-only process for explicit physics simulations.

Only joint/pose JSON enters; no SDK, camera, execute_push or hardware profile.
Physics lives in its existing separate Python environment, without installations.
"""
import argparse
import contextlib
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.events import StopToken


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True,type=Path);parser.add_argument('--directory',required=True,type=Path)
    args=parser.parse_args();data=json.loads(args.profile.read_text());wire=sys.stdout
    def send(value):wire.write(json.dumps(value,allow_nan=False)+'\n');wire.flush()
    backend=None
    try:
        with contextlib.redirect_stdout(sys.stderr):
            from rm75_app.planning.backends.curobo2 import Curobo2Backend,Curobo2BackendConfig
            from rm75_app.pusht.motion import CuroboPushExecutor
            from rm75_app.pusht.model import Config,Push
            from rm75_app.pusht.observation import Observation
            from rm75_app.planning.contracts import CollisionObject,Pose,PlanningScene
            options=dict(data.get('planner',{}))
            for key in ('robot_config','curobo_root'):
                if key in options and options[key] is not None:options[key]=Path(options[key])
            backend=Curobo2Backend(Curobo2BackendConfig(**options))
            native=backend._ensure_planner()
            q=native.default_joint_state.position.reshape(-1)[:7].detach().cpu().numpy()
            backend.set_gripper_collision_state(closed=True)
            geometry=backend.closed_gripper_tool_geometry(q)
            periodic_audits=[]
            solve_variants=backend.solve_pose_ik_variants
            def audited_variants(request):
                result=solve_variants(request)
                periodic_audits.append(dict(backend._stage_ik_periodic_audit))
                return result
            backend.solve_pose_ik_variants=audited_variants
            import yaml
            robot_cfg=yaml.safe_load(Path(backend.config.robot_config).read_text())['robot_cfg']['kinematics']
            send(dict(ready=True,initial_q=q.tolist(),spheres=geometry.spheres.tolist(),links=list(geometry.links),
                urdf=robot_cfg['urdf_path'],
                gripper_locks={name:backend.config.gripper_collision_closed_joint_position
                               for name in robot_cfg['lock_joints']},execute_real=False))
            class ArmState:
                hz=30
                def read_joints(self):return q.copy()
                def execute(self,*a,**k):raise AssertionError('Planning process cannot execute')
            count=0
            for line in sys.stdin:
                request=json.loads(line)
                if request.get('op')=='close':break
                if request.get('op')!='plan':raise ValueError('Only planning is allowed')
                q=np.asarray(request['q'],dtype=float)
                if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Invalid simulated joints')
                observation=Observation.from_dict(request['observation'])
                if observation.source!='simulation':raise ValueError('Only explicit simulation observations accepted')
                push=Push(**request['push']);events=[];tick=time.monotonic();periodic_audits.clear()
                class Events:
                    def emit(self,event,**values):events.append(dict(event=event,**values))
                executor=CuroboPushExecutor(backend,ArmState(),Config.from_dict(data['model']),data['motion'],
                    StopToken(args.directory/'STOP'),Events(),None)
                result=dict(execute_real=False,hardware_connected=False,hardware_profile_qualified=False,
                    complete_chain=False,validation_success=False,events=events,source_observation=request['observation'])
                try:
                    prepared=executor.plan_push(push,observation)
                    result.update(complete_chain=True,validation_success=True,
                        stages=[dict(stage=s,positions=p.tolist(),times=t.tolist()) for s,p,t in prepared.stages])
                except Exception as exc:result['error']=f'{type(exc).__name__}: {exc}'
                result['periodic_ik_audit']=list(periodic_audits)
                result['elapsed_s']=time.monotonic()-tick;count+=1
                path=args.directory/f'plan_{count:03d}.json';atomic_json(path,result)
                send(dict(result=str(path),success=result['complete_chain'],elapsed_s=result['elapsed_s']))
    except BaseException as exc:
        send(dict(ready=False,error=f'{type(exc).__name__}: {exc}'))
        raise
    finally:
        if backend is not None:backend.__exit__(None,None,None)


if __name__=='__main__':main()
