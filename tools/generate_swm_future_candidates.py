#!/usr/bin/env python3
"""Generate new original native candidates from a historical measured seed."""
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('profile','transition','task-request','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    from tools.run_network_isolated import block_network
    block_network()
    def read(path):
        if path.stat().st_size>16000000:raise ValueError('Candidate input exceeds budget')
        return json.loads(path.read_bytes())
    profile=read(args.profile);transition=read(args.transition);task=read(args.task_request)
    if task.get('task')!='pusht':raise ValueError('Original PushT task request required')
    snapshot=transition['initial_snapshot'];args.output.mkdir(parents=True,exist_ok=False)
    deadline=time.monotonic()+180;events=[]
    def check():
        if time.monotonic()>deadline:raise TimeoutError('Original candidate generation budget exceeded')
    def emit(kind,**row):
        if kind=='retreat_contact_escape_audited':events.append(dict(kind=kind,**row))
    from rm75_app.planning.backends.curobo2 import Curobo2Backend,Curobo2BackendConfig
    from rm75_app.pusht.model import Config
    from rm75_app.pusht.motion import CuroboPushExecutor
    from tools.pusht_physics_planner import audit_predicted_retreat
    class AuditedPhysicsPushExecutor(CuroboPushExecutor):
        def plan_push(self,push,observation,**kwargs):
            prepared=super().plan_push(push,observation,**kwargs)
            audit_predicted_retreat(self,prepared,push,observation)
            return prepared
    from rm75_app.swm.future_candidates import generate_original_push_candidates
    from rm75_app.swm.scene import digest
    report=dict(scope='new_original_candidates_from_historical_measured_snapshot',source_snapshot_id=snapshot['snapshot_id'],
                hardware_connected=False,execution_authorized=False,live_observation_refreshed=False)
    try:
        with Curobo2Backend(Curobo2BackendConfig(**profile['planner'])) as backend:
            arm=SimpleNamespace(hz=30.,read_joints=lambda:np.asarray(snapshot['robot']['positions'],float).copy())
            executor=AuditedPhysicsPushExecutor(backend,arm,Config.from_dict(profile['model']),profile['motion'],
                SimpleNamespace(check=check),SimpleNamespace(emit=emit),None)
            candidates=generate_original_push_candidates(executor,snapshot,task['parameters']['goal_pose'])
        if len(events)<2*len(profile['model']['friction_scales']):
            raise ValueError('Original predicted retreat escape audit evidence missing')
        report['candidates']=[]
        for index,candidate in enumerate(candidates):
            atomic_json(args.output/f'candidate_{index}.json',candidate)
            report['candidates'].append(dict(index=index,plan_digest=digest(candidate),selected_push=candidate['selected_push'],
                samples=sum(len(row['positions']) for row in candidate['stages'])))
        report.update(status='CANDIDATES_GENERATED_NOT_EXECUTION_AUTHORIZED',retreat_audits=events)
    except Exception as exc:report.update(status='REJECTED',error_type=type(exc).__name__,reason=str(exc)[:4096])
    atomic_json(args.output/'summary.json',report);print(json.dumps(report),flush=True)
    return 1 if report['status']=='REJECTED' else 0


if __name__=='__main__':raise SystemExit(main())
