#!/usr/bin/env python3
"""Run a frozen native SIM through the actual workcell service/worker; never real."""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json,read_json
from rm75_app.workcell.service import WorkcellService
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.magnetic.design import validate_design
from rm75_app.workcell.native_outcome import NativeOutcomeCapture


def read_original_task_bundle(directory):
    """Read one existing task dependency closure; never copy/modify old files."""
    manifest=Path(directory).resolve()
    if manifest.is_dir():manifest=manifest/'manifest.json'
    data=read_json(manifest)
    if data.get('schema')!='jimu_task_manifest_v1':raise ValueError('Unknown original Jimu manifest schema')
    files={'manifest':manifest}
    for name,key in (('builder','builder_scene_json'),('fixed_scene','sam6d_fixed_scene_result_file')):
        value=data.get(key)
        if not isinstance(value,str) or not value:raise ValueError('Task manifest dependency missing: '+key)
        path=(manifest.parent/value).resolve()
        if not path.is_relative_to(manifest.parent) or not path.is_file():
            raise ValueError('Task dependency must be an existing file inside its original task directory')
        files[name]=path
    validate_design(read_json(files['builder']))
    fixed=read_json(files['fixed_scene'])
    if not isinstance(fixed.get('results'),list) or not fixed['results']:
        raise ValueError('Original task must provide nonempty frozen camera results')
    return files,{key:hashlib.sha256(path.read_bytes()).hexdigest() for key,path in files.items()}


def native_outcomes(text, expected_cycles):
    """Only the original main-loop markers count; prefetch episodes do not."""
    capture=NativeOutcomeCapture(io.StringIO())
    capture.write(text+'\n')
    return capture.report(expected_cycles)


def read_documented_jimu_start(path):
    """Read ONLY seven numeric start angles, never execute a documentation command."""
    path=Path(path).resolve();raw=path.read_bytes()
    matches=re.findall(r'--jimu-sim-start-joints-deg[ \t]+([^\r\n\\]+)',raw.decode('utf-8'))
    vectors=[]
    for match in matches:
        values=tuple(float(value) for value in match.split())
        if len(values)!=7 or not all(math.isfinite(value) for value in values):
            raise ValueError('Documented Jimu start must contain seven finite angles')
        vectors.append(values)
    if not vectors or len(set(vectors))!=1:
        raise ValueError('Document must name one unambiguous original Jimu start configuration')
    return {'source_name':path.name,'source_sha256':hashlib.sha256(raw).hexdigest(),
            'joints_deg':list(vectors[0]),'read_only':True}


def jimu_lift_trial_args(task, enabled):
    """User-approved SIM comparison: only the failed joint-search lift +3 mm.

    The pinned native default is 0.100 m. Keep the independent post-grasp lift,
    return/placement heights, geometry, candidates and collision policy intact.
    This opt-in runner never updates the production or hardware profile.
    """
    if not enabled:
        return []
    if task != 'magnetic':
        raise ValueError('The +3 mm lift trial is Jimu SIM only')
    return ['--joint-search-start-collision-lift-m', '0.103']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',choices=('pickplace','magnetic'),required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--extensions',type=Path,required=True)
    parser.add_argument('--design',type=Path)
    parser.add_argument('--task-dir',type=Path,help='Read-only original Jimu manifest + builder + frozen poses, without migration')
    parser.add_argument('--jimu-start-command-doc',type=Path,
        help='Separate SIM comparison: read seven start angles from original command documentation')
    parser.add_argument('--audit-roof-ik',action='store_true',
        help='Opt-in read-only first roof IK batch per original phase/source; no extra solve')
    parser.add_argument('--jimu-lift-plus-3mm',action='store_true',
        help='Jimu SIM only: test the original failed joint-search lift at 103 instead of 100 mm')
    parser.add_argument('--audit-current-table-failures',action='store_true',
        help='Read-only failed lift / already-generated tennis reverse path; no new solve or selection')
    parser.add_argument('--tennis-ik-review',choices=('baseline','standard-reference','continuation','yaw-midpoints'),
        help='Explicit Tennis frozen SIM: FK/IK + center contracts and fixed 32-seed reference comparison')
    parser.add_argument('--record-sim-video',action='store_true',
        help='Record original SIM motion windows at scale 1; adds render time, never real-time qualification')
    inputs=parser.add_mutually_exclusive_group()
    inputs.add_argument('--fixed-sam6d',type=Path)
    inputs.add_argument('--fixed-world',type=Path,help='Original T_world_obj scene; PickPlace SIM direct entry only')
    parser.add_argument('--object-name',default='gluestick')
    parser.add_argument('--cycle-order',nargs='+',help='Explicit original same-scene frozen SIM object order')
    parser.add_argument('--stop-after-native-start',action='store_true')
    parser.add_argument('--stop-after-transport-audit',action='store_true',
        help='Cancel only after the actual GPU transport has passed its complete sampled-path audit')
    parser.add_argument('--stop-after-collision-diagnostic',action='store_true',
        help='Bounded diagnosis: cancel after the first complete read-only GPU collision snapshot')
    parser.add_argument('--stop-after-return-diagnostic',action='store_true',
        help='Cancel after the original failed return query records start/goal collision evidence')
    parser.add_argument('--stop-after-release-observation',action='store_true',
        help='Cancel after observing the actual Jimu post-release execution boundary')
    parser.add_argument('--timeout-s',type=float,default=600.)
    args=parser.parse_args()
    if args.cycle_order and (args.task!='pickplace' or args.fixed_world is None
            or args.cycle_order[0]!=args.object_name):
        parser.error('Cycle order requires native-world PickPlace SIM and first source equal to object-name')
    if args.tennis_ik_review and (args.task!='pickplace' or 'tennis' not in (args.cycle_order or [args.object_name])
            or args.fixed_world is None or args.fixed_world.resolve()!=ROOT/'assets/test_scenes/current_table.json'):
        parser.error('Tennis IK review requires the original current-table Tennis SIM')
    if args.jimu_lift_plus_3mm and args.task!='magnetic':parser.error('The +3 mm lift trial is Jimu SIM only')
    if args.audit_roof_ik and args.task!='magnetic':parser.error('Roof IK diagnostics are Jimu SIM only')
    if args.audit_current_table_failures and (args.task!='pickplace'
            or not set(args.cycle_order or [args.object_name]) & {'gluestick','hongshupian','tennis'}
            or args.fixed_world is None
            or args.fixed_world.resolve()!=ROOT/'assets/test_scenes/current_table.json'):
        parser.error('Focused diagnostics require one of the three reviewed current-table native-world sources')
    root=ROOT/'rm75_app/_vendor/working_snapshot';verify_snapshot(root)
    bundle=None;bundle_hashes=None
    if args.task_dir:
        if args.task!='magnetic':parser.error('Original task bundle is Jimu only')
        if any(value is not None for value in (args.design,args.fixed_sam6d,args.fixed_world)):
            parser.error('Task bundle must not be mixed with overridden builder/fixed inputs')
        bundle,bundle_hashes=read_original_task_bundle(args.task_dir)
        args.design=bundle['builder'];args.fixed_sam6d=bundle['fixed_scene']
    if not 1<=args.timeout_s<=900:raise ValueError('Timeout must be within 1..900 seconds')
    if args.task=='pickplace' and args.fixed_sam6d is None and args.fixed_world is None:
        parser.error('PickPlace requires an explicit existing frozen input; no camera fallback')
    if args.task!='pickplace' and args.fixed_world is not None:parser.error('World input is PickPlace SIM only')
    if args.task=='magnetic' and args.design is None:parser.error('Supply the original full builder design')
    documented_start=None
    if args.jimu_start_command_doc:
        if args.task!='magnetic':parser.error('Documented start comparison is Jimu SIM only')
        documented_start=read_documented_jimu_start(args.jimu_start_command_doc)
    fixed=(args.fixed_world or args.fixed_sam6d or root/'Beta_demo-codex-v0.9/jimu_portable_repro/scenes/jimu_assembly_anchors_default_sam6d.json').resolve()
    data=read_json(fixed)
    field,kind=('objects',dict) if args.fixed_world else ('results',list)
    if not isinstance(data.get(field),kind):raise ValueError('Input does not match selected original frozen schema')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    profile=read_json(ROOT/'examples/workcell/machine.example.json')
    section=profile[args.task]
    if args.cycle_order:section['frozen_source_order']=args.cycle_order
    if args.audit_roof_ik:section['audit_roof_ik']=True
    if args.audit_current_table_failures:section['audit_current_table_failures']=True
    if args.tennis_ik_review:section['tennis_ik_review']=args.tennis_ik_review
    if args.record_sim_video:section['record_sim_video']=True
    section.update(python=str(Path(sys.executable).resolve()),render_mode='none',fixed_scene=str(fixed),
        fixed_scene_format='native_world' if args.fixed_world else 'sam6d',
        simulation_contact_policy='transport_world_checked_compatibility')
    section['native_args']=['--curobo-torch-extensions-dir',str(args.extensions.resolve()),
        '--camera-extrinsic-opencv-path',str(ROOT/'assets/calibration/camera_extrinsic_opencv.npy')]
    section['native_args']+=jimu_lift_trial_args(args.task,args.jimu_lift_plus_3mm)
    if args.task=='magnetic':
        section['native_args']+=['--jimu-build-layers','two','--jimu-second-layer-triangle-profile',
                                 '--no-jimu-demo-triangle-apriltag']
        if bundle:section['native_args']+=['--jimu-task-dir',str(bundle['manifest'])]
        if documented_start:
            section['native_args']+=['--jimu-sim-start-joints-deg',*[str(q) for q in documented_start['joints_deg']]]
        params={'design':read_json(args.design)}
    else:params={'object_name':args.object_name}
    expected_cycles=len(validate_design(params['design']).ordered_roles) if args.task=='magnetic' else len(args.cycle_order or [args.object_name])
    assert profile['hardware']['hardware_reviewed'] is False and section['integration_qualified'] is False
    atomic_json(output/'machine.json',profile)
    spec={'task':args.task,'mode':'sim','parameters':params};atomic_json(output/'request.json',spec)
    report=dict(task=args.task,mode='sim',execute_real=False,hardware_connected=False,
        verified_task_success=None,fixed_input_sha256=hashlib.sha256(fixed.read_bytes()).hexdigest(),
        stopped_for_validation=False,completed=False)
    report['adapter_source_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted((ROOT/'rm75_app/workcell').glob('*.py'))}
    report['jimu_joint_search_lift_trial_m']=.103 if args.jimu_lift_plus_3mm else None
    report['original_task_bundle_sha256']=bundle_hashes
    report['original_task_bundle_read_only']=bool(bundle)
    report['documented_start_comparison']=documented_start
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
            diagnostic=next((row.get('evidence',{}) for row in state.get('events',[])
                if row.get('kind')=='contact_audit' and row.get('evidence',{}).get('event')=='jimu_read_only_collision_diagnostic'
                and row['evidence'].get('geometry_detail_recorded')),None)
            video_sealed=(not args.record_sim_video or
                (service.root/'jobs'/job/'sim_video/recording.json').is_file())
            if args.stop_after_collision_diagnostic and diagnostic and video_sealed:
                report['diagnostic_trigger']={k:diagnostic.get(k) for k in ('step_id','status','scene_fingerprint')}
                report['stop_result']=service.cancel(job);report['stopped_for_validation']=True;break
            return_diag=next((row.get('evidence',{}) for row in state.get('events',[])
                if row.get('kind')=='contact_audit' and row.get('evidence',{}).get('event')=='jimu_return_query_diagnostic'),None)
            if args.stop_after_return_diagnostic and return_diag:
                report['return_diagnostic_trigger']={key:return_diag.get(key) for key in
                    ('step_id','source','native_status','diagnostic_complete','state_unchanged')}
                report['stop_result']=service.cancel(job);report['stopped_for_validation']=True;break
            release_obs=next((row.get('evidence',{}) for row in state.get('events',[])
                if row.get('kind')=='contact_audit' and row.get('evidence',{}).get('event')=='jimu_release_execution_observation'),None)
            if args.stop_after_release_observation and release_obs:
                report['release_observation_trigger']={key:release_obs.get(key) for key in
                    ('step_id','source','diagnostic_complete','state_unchanged','max_gripper_model_error_rad')}
                report['stop_result']=service.cancel(job);report['stopped_for_validation']=True;break
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
        if bundle:
            report['original_task_bundle_sha256_after']={key:hashlib.sha256(path.read_bytes()).hexdigest()
                for key,path in bundle.items()}
            report['original_task_bundle_unchanged']=report['original_task_bundle_sha256_after']==bundle_hashes
        if documented_start:
            report['documented_start_source_unchanged']=(hashlib.sha256(args.jimu_start_command_doc.read_bytes()).hexdigest()
                ==documented_start['source_sha256'])
        report['completed']=bool(report.get('command_completed') and report.get('native_full_chain_passed')
            and report.get('original_task_bundle_unchanged',True) and report.get('documented_start_source_unchanged',True))
        if args.audit_roof_ik:
            from rm75_app.workcell.jimu_roof_ik_diagnostics import roof_audit_status
            events=ROOT/'runtime_data/workcell/jobs'/job/'events.jsonl' if job else None
            diagnostic_rows=[]
            if events is not None and events.is_file():
                for line in events.read_text().splitlines():
                    event=json.loads(line)
                    if (event.get('kind')=='contact_audit' and
                            event.get('evidence',{}).get('event')=='jimu_roof_ik_batch_diagnostic'):
                        diagnostic_rows.append(event['evidence'])
            expected_roofs=[role for role in validate_design(params['design']).ordered_roles
                            if 'roof_triangle' in role]
            report['roof_ik_audit']=roof_audit_status(diagnostic_rows,expected_roofs)
            report['completed']=bool(report['completed'] and report['roof_ik_audit']['passed'])
        atomic_json(output/'result.json',report)
    print(json.dumps({k:report.get(k) for k in ('job_id','completed','stopped_for_validation','elapsed_s','error')}))
    return 0 if report['completed'] or (report['stopped_for_validation'] and
        report.get('job',{}).get('status')=='cancelled') else 42


if __name__=='__main__':raise SystemExit(main())
