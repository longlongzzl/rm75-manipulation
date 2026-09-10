#!/usr/bin/env python3
"""Generate or run a serial, reproducible PushT physics request matrix.

Generation is the default. --run uses the real WorkcellService but never real
mode. Each case has an independent wall timeout and retained failure result.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time
from rm75_app.workcell.io import read_json,atomic_json,finite,integer
from rm75_app.pusht.scenarios import sample_scenarios


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--app-root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--count',type=int,default=12)
    parser.add_argument('--geometry-id',default='original')
    parser.add_argument('--backend',choices=['tool_only_physics','full_arm_physics'],default='full_arm_physics')
    parser.add_argument('--timeout-s',type=float,default=600.)
    parser.add_argument('--run',action='store_true')
    args=parser.parse_args(argv)
    finite(args.timeout_s,'timeout_s',10,3600)
    if args.output.exists():raise FileExistsError('Do not overwrite previous matrix/evidence')
    profile=read_json(args.profile)
    suite=sample_scenarios(profile,seed=args.seed,count=args.count,geometry_id=args.geometry_id)
    args.output.mkdir(parents=True)
    atomic_json(args.output/'cases.json',suite)
    if not args.run:
        print(json.dumps({'generated':len(suite['cases']),'ran':0,'hardware_connected':False}));return 0
    from rm75_app.workcell.service import WorkcellService
    service=WorkcellService(args.app_root,args.profile,allow_real=False)
    results=[]
    try:
        for case in suite['cases']:
            spec=dict(task='pusht',mode='sim',parameters=dict(initial_pose=case['initial_pose'],
                goal_pose=case['goal_pose'],geometry_id=args.geometry_id,simulation_backend=args.backend,
                run_until_goal=True,maximum_push_length_m=profile.get('pusht',{}).get('model',{}).get('maximum_push_length_m',.05)))
            row={'case_id':case['case_id'],'state_digest':case['state_digest'],'request':spec,'success':False}
            try:
                started=time.monotonic();job=service.submit(spec);ident=job['job_id'];row['job_id']=ident
                while time.monotonic()-started<args.timeout_s:
                    value=service.job(ident)
                    if value['status']!='running':break
                    time.sleep(.25)
                else:
                    service.cancel(ident)
                    if service._thread:service._thread.join(timeout=10)
                    value=service.job(ident);row['timed_out']=True
                row.update(elapsed_s=time.monotonic()-started,result=value.get('result'),
                           success=value.get('result',{}).get('task_success') is True and not row.get('timed_out',False))
                # The wait thread releases the service active reference under its lock.
                if service._thread:service._thread.join(timeout=10)
                if service.active:raise RuntimeError('Previous worker did not terminate; refusing to overlap GPU jobs')
            except Exception as exc:
                row.update(error_type=type(exc).__name__,error=str(exc))
                if service.active:
                    service.cancel(service.active)
                    if service._thread:service._thread.join(timeout=10)
                    if service.active:
                        results.append(row);break
            results.append(row)
            atomic_json(args.output/'results.json',dict(requested=len(suite['cases']),attempted=len(results),
                successes=sum(x['success'] for x in results),results=results,hardware_connected=False))
    finally:
        service.close()
        atomic_json(args.output/'results.json',dict(requested=len(suite['cases']),attempted=len(results),
            not_run=[c['case_id'] for c in suite['cases'][len(results):]],
            successes=sum(x['success'] for x in results),results=results,hardware_connected=False))
    return 0 if len(results)==len(suite['cases']) and all(x['success'] for x in results) else 1

if __name__=='__main__':raise SystemExit(main())
