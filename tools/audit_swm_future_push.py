#!/usr/bin/env python3
"""Audit dense original PushT candidate predictions with the original GPU model."""
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('profile','transition','future-motion','prediction','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    from tools.run_network_isolated import block_network
    block_network()
    def read(path):
        if path.stat().st_size>16000000:raise ValueError('Native audit input exceeds budget')
        return json.loads(path.read_bytes())
    profile=read(args.profile);transition=read(args.transition);motion=read(args.future_motion);prediction=read(args.prediction)
    args.output.mkdir(parents=True,exist_ok=False);last={};deadline=time.monotonic()+180
    def check():
        if time.monotonic()>deadline:raise TimeoutError('Native future audit budget exceeded')
    from rm75_app.planning.backends.curobo2 import Curobo2Backend,Curobo2BackendConfig
    from rm75_app.pusht.motion import CuroboPushExecutor
    from rm75_app.pusht.model import Config
    from rm75_app.swm.future_push_audit import audit_future_push
    result=dict(scope='original_dynamic_target_GPU_collision_audit',hardware_connected=False,
                future_motion_digest=motion['future_motion_digest'],hypothesis_id=prediction['hypothesis_id'])
    try:
        with Curobo2Backend(Curobo2BackendConfig(**profile['planner'])) as backend:
            executor=CuroboPushExecutor(backend,None,Config.from_dict(profile['model']),profile['motion'],
                SimpleNamespace(check=check),SimpleNamespace(emit=lambda *a,**k:None),None)
            executor.names=tuple(transition['initial_snapshot']['robot']['joint_names'])
            result['audit']=audit_future_push(executor,motion,prediction,transition['initial_snapshot'],transition['object_id'],
                                             progress=lambda row:last.update(row))
        result['status']='COLLISION_SAMPLES_PASSED_NOT_EXECUTION_AUTHORIZED'
    except Exception as exc:
        result.update(status='REJECTED',error_type=type(exc).__name__,reason=str(exc)[:4096],last_sample=last)
    atomic_json(args.output/'summary.json',result);print(json.dumps(result),flush=True)
    return 1 if result['status']=='REJECTED' else 0


if __name__=='__main__':raise SystemExit(main())
