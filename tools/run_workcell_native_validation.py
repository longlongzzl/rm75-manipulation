#!/usr/bin/env python3
"""Run a frozen native SIM through the actual workcell service/worker; never real."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json,read_json
from rm75_app.workcell.service import WorkcellService
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.magnetic.design import validate_design
from rm75_app.workcell.native_outcome import NativeOutcomeCapture


def native_outcomes(text, expected_cycles):
    """Only the original main-loop markers count; prefetch episodes do not."""
    capture=NativeOutcomeCapture(io.StringIO())
    capture.write(text+'\n')
    return capture.report(expected_cycles)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',choices=('pickplace','magnetic'),required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--extensions',type=Path,required=True)
    parser.add_argument('--design',type=Path)
    parser.add_argument('--fixed-sam6d',type=Path)
    parser.add_argument('--object-name',default='gluestick')
    parser.add_argument('--stop-after-native-start',action='store_true')
    parser.add_argument('--stop-after-transport-audit',action='store_true',
        help='Cancel only after the actual GPU transport has passed its complete sampled-path audit')
    parser.add_argument('--timeout-s',type=float,default=600.)
    args=parser.parse_args()
    root=ROOT/'rm75_app/_vendor/working_snapshot';verify_snapshot(root)
    if not 1<=args.timeout_s<=900:raise ValueError('Timeout must be within 1..900 seconds')
    if args.task=='pickplace' and args.fixed_sam6d is None:
        parser.error('PickPlace requires an explicit existing frozen SAM6D result; no camera fallback')
    if args.task=='magnetic' and args.design is None:parser.error('Supply the original full builder design')
    fixed=(args.fixed_sam6d or root/'Beta_demo-codex-v0.9/jimu_portable_repro/scenes/jimu_assembly_anchors_default_sam6d.json').resolve()
    data=read_json(fixed)
    if not isinstance(data.get('results'),list):raise ValueError('Expected original SAM6D result schema')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    profile=read_json(ROOT/'examples/workcell/machine.example.json')
    section=profile[args.task]
    section.update(python=str(Path(sys.executable).resolve()),render_mode='none',fixed_scene=str(fixed),
        simulation_contact_policy='transport_world_checked_compatibility')
    section['native_args']=['--curobo-torch-extensions-dir',str(args.extensions.resolve()),
        '--camera-extrinsic-opencv-path',str(ROOT/'assets/calibration/camera_extrinsic_opencv.npy')]
    if args.task=='magnetic':
        section['native_args']+=['--jimu-build-layers','two','--jimu-second-layer-triangle-profile',
                                 '--no-jimu-demo-triangle-apriltag']
        params={'design':read_json(args.design)}
    else:params={'object_name':args.object_name}
    expected_cycles=len(validate_design(params['design']).ordered_roles) if args.task=='magnetic' else 1
    assert profile['hardware']['hardware_reviewed'] is False and section['integration_qualified'] is False
    atomic_json(output/'machine.json',profile)
    spec={'task':args.task,'mode':'sim','parameters':params};atomic_json(output/'request.json',spec)
    report=dict(task=args.task,mode='sim',execute_real=False,hardware_connected=False,
        verified_task_success=None,fixed_input_sha256=hashlib.sha256(fixed.read_bytes()).hexdigest(),
        stopped_for_validation=False,completed=False)
    service=WorkcellService(ROOT,output/'machine.json',allow_real=False)
    started=time.monotonic();job=None
    try:
        job=service.submit(spec)['job_id'];report['job_id']=job
        print(json.dumps({'job_id':job,'task':args.task,'mode':'sim'}),flush=True)
        while service.active:
            state=service.job(job)
            if state.get('input_request'):
                report['unanswered_native_input']=state['input_request'];service.cancel(job);break
            if args.stop_after_native_start and state.get('progress',{}).get('kind','').startswith('legacy_profile_'):
                report['stop_result']=service.cancel(job);report['stopped_for_validation']=True;break
            audited=next((row.get('evidence',{}) for row in state.get('events',[])
                if row.get('kind')=='contact_audit' and row.get('evidence',{}).get('event')=='transport_full_world_audit'
                and row['evidence'].get('samples',0)>0),None)
            if args.stop_after_transport_audit and audited:
                report['stop_trigger']={k:audited[k] for k in ('event','samples','step_id','payload_spheres')}
                stop_at=time.monotonic();report['stop_result']=service.cancel(job)
                report['stop_request_elapsed_s']=time.monotonic()-stop_at
                report['stopped_for_validation']=True;break
            if time.monotonic()-started>args.timeout_s:
                report['timeout']=True;service.cancel(job);break
            time.sleep(.1)
        service.close();report['job']=service.job(job)
        report['command_completed']=report['job']['status']=='command_completed_unverified'
    except BaseException as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
    finally:
        service.close();report['elapsed_s']=time.monotonic()-started
        if job:
            stdout=ROOT/'runtime_data/workcell/jobs'/job/'stdout.log'
            if stdout.is_file():report.update(native_outcomes(stdout.read_text(errors='replace'),expected_cycles))
        report['completed']=bool(report.get('command_completed') and report.get('native_full_chain_passed'))
        atomic_json(output/'result.json',report)
    print(json.dumps({k:report.get(k) for k in ('job_id','completed','stopped_for_validation','elapsed_s','error')}))
    return 0 if report['completed'] or (report['stopped_for_validation'] and
        report.get('job',{}).get('status')=='cancelled') else 42


if __name__=='__main__':raise SystemExit(main())
