#!/usr/bin/env python3
"""Exercise the original native factory through the existing hypothesis planner."""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('profile','transition','geometry','urdf','task-request','hypotheses','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--python',required=True)
    parser.add_argument('--cancel-after-first-native-audit',action='store_true',
        help='Explicit lifecycle probe: interrupt after an actual original GPU collision audit')
    args=parser.parse_args()
    from tools.run_network_isolated import block_network
    block_network()
    from rm75_app.swm.native_push_hypotheses import NativePushHypothesisFactory
    from rm75_app.swm.planning import ParallelHypothesisPlanner
    from rm75_app.swm.skills import SkillRequest
    from rm75_app.workcell.io import atomic_json
    def read(path):
        if path.stat().st_size>16000000:raise ValueError('Native factory input exceeds budget')
        return json.loads(path.read_bytes())
    profile=read(args.profile);transition=read(args.transition);geometry=read(args.geometry)
    task=read(args.task_request);hypotheses=read(args.hypotheses)
    if task.get('task')!='pusht':raise ValueError('Original PushT task required')
    snapshot=transition['initial_snapshot'];oid=transition['object_id'];goal=task['parameters']['goal_pose']
    target=np.asarray(snapshot['objects'][oid]['measured']['T_world_object'],float).copy()
    c,s=np.cos(goal[2]),np.sin(goal[2]);target[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];target[:2,3]=goal[:2]
    request=SkillRequest('push',oid,target=target);args.output.mkdir(parents=True,exist_ok=False)
    deadline=time.monotonic()+1500
    def check():
        if time.monotonic()>deadline:raise TimeoutError('Native hypothesis transaction budget exceeded')
    report=dict(scope='actual_original_native_factory_through_ParallelHypothesisPlanner',
        online_worker_consumption=False,live_observation_refreshed=False,hardware_connected=False,
        execution_authorized=False,full_primitive_audit_issued=False)
    native_audit_completed=False;factory=None
    class CancellationProbeFactory(NativePushHypothesisFactory):
        @contextmanager
        def _executor(self,snapshot):
            nonlocal native_audit_completed
            with super()._executor(snapshot) as executor:
                original=executor._audit
                def cancel_after_audit(*values,**kwargs):
                    result=original(*values,**kwargs)
                    native_audit_completed=True
                    raise InterruptedError('Explicit cancellation after completed original GPU audit')
                executor._audit=cancel_after_audit
                yield executor
    factory_type=CancellationProbeFactory if args.cancel_after_first_native_audit else NativePushHypothesisFactory
    try:
        with factory_type(profile,geometry,args.urdf,python=args.python,
                directory=args.output,check=check) as factory:
            plan,evidence=ParallelHypothesisPlanner(factory,workers=1,max_candidates=2,check=check).solve(request,snapshot,hypotheses)
            report.update(status='NATIVE_PLAN_SELECTED_NOT_EXECUTION_AUTHORIZED',plan_id=plan.payload_digest,
                source_snapshot_id=plan.source_snapshot_id,planner=plan.planner,
                expected_object_pose=plan.expected_object_pose,evidence=evidence)
        report['private_factory_closed']=factory.closed
    except Exception as exc:
        report.update(status='REJECTED',error_type=type(exc).__name__,reason=str(exc)[:4096])
        if args.cancel_after_first_native_audit and native_audit_completed and isinstance(exc,InterruptedError):
            report.update(status='CANCELLED_AFTER_NATIVE_AUDIT_NOT_EXECUTED',
                scope='explicit_original_GPU_audit_boundary_cancellation',native_audit_completed=True)
    finally:
        if factory is not None:
            report.update(private_factory_closed=factory.closed,
                private_directory_removed=not factory.directory.exists())
    atomic_json(args.output/'summary.json',report);print(json.dumps(report),flush=True)
    return 1 if report['status']=='REJECTED' else 0


if __name__=='__main__':raise SystemExit(main())
