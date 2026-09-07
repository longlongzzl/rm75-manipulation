#!/usr/bin/env python3
"""Read a saved synthetic PushT fixture; diagnose its original descent on GPU."""
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import traceback

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from run_pusht_gpu_validation import RecordedBackend
from rm75_app.planning.backends.curobo2 import Curobo2BackendConfig
from rm75_app.planning.contracts import BatchPlanningRequest,JointConfiguration,Pose,PoseCandidate
from rm75_app.pusht.model import Config,Push
from rm75_app.pusht.motion import CuroboPushExecutor
from rm75_app.pusht.observation import Observation
from rm75_app.workcell.events import StopToken
from rm75_app.workcell.io import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();data=json.loads(args.input.read_text())
    if (data.get('scenario')!='authored_simulation_fixture' or data.get('hardware_connected') is not False
        or data.get('fixture')!='low_table' or data.get('case')!='orthogonal_tool'):
        raise ValueError('Only the saved low-table orthogonal-tool fixture is accepted')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    report=dict(execute_real=False,hardware_connected=False,purpose='full-world original descent diagnostic',
                complete_chain=False,fixture=data['fixture'],case=data['case'])
    cfg=Config.from_dict(data['config']);motion=data['motion'];push=Push(**data['push'])
    backend=RecordedBackend(Curobo2BackendConfig())
    try:
        backend.set_gripper_collision_state(closed=True)
        native=backend._ensure_planner();names=tuple(native.joint_names)
        q=native.default_joint_state.position.reshape(-1)[:7].detach().cpu().numpy()
        executor=CuroboPushExecutor(backend,None,cfg,motion,StopToken(out/'STOP'),None,None)
        obs=Observation('synthetic_gpu_fixture',1,time.time(),(.35,-.18,0.),'simulation')
        scene=executor._scene(obs);backend.update_scene(scene)
        entry=np.asarray(push.contact)-np.asarray(push.direction)*cfg.approach_gap_m
        xyz=[*entry,motion['push_tcp_z_m']+motion['hover_clearance_m']]
        orientation=motion['tool_quaternion_wxyz']
        candidate=PoseCandidate('approach',Pose(xyz,orientation))
        request=BatchPlanningRequest(JointConfiguration(names,q),(candidate,),scene)
        approach=backend.plan_candidates(request).best((candidate,))
        if approach is None: raise RuntimeError('original approach failed')
        q=approach.trajectory.positions[-1]
        start=backend.tool_pose_for_configuration(JointConfiguration(names,q),'gripper_tcp')
        target=np.array([*entry,motion['push_tcp_z_m']])
        positions=np.linspace(start.position,target,17)
        candidates=tuple(PoseCandidate(f'descent_{i:02}',Pose(xyz,orientation)) for i,xyz in enumerate(positions))
        backend.prepare_pose_candidates_coarse(candidates,scene)
        report['original_line_positions']=positions.tolist()
        report['coarse_metrics']=backend.pose_candidate_metrics(candidates)
        for scale in (.5,1.,2.,4.):
            request=BatchPlanningRequest(JointConfiguration(names,q),(candidates[-1],),scene)
            result=backend.plan_linear_candidates(request,axis='z',project_distance_to_goal=False,
                                                  non_terminal_scale=scale).best((candidates[-1],))
            row=dict(scale=scale,success=result is not None)
            if result is not None:
                path=result.trajectory.positions
                poses=[backend.tool_pose_for_configuration(JointConfiguration(names,p),'gripper_tcp') for p in path]
                actual=np.array([p.position for p in poses]);delta=target-start.position
                t=np.clip((actual-start.position)@delta/(delta@delta),0,1)
                gap=np.linalg.norm(actual-(start.position+t[:,None]*delta),axis=1)
                row.update(max_gap_m=float(gap.max()),tcp_positions=actual.tolist(),q=path.tolist(),
                           worst_index=int(gap.argmax()),samples=len(path))
            report.setdefault('linear_scale_diagnostic',[]).append(row)
    except Exception as exc:
        report.update(error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
    finally:
        report['backend_records']=backend.records
        backend.__exit__(None,None,None);atomic_json(out/'result.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in
        ('backend_records','coarse_metrics','original_line_positions','linear_scale_diagnostic')}))
    return int('error' in report)


if __name__=='__main__':raise SystemExit(main())
