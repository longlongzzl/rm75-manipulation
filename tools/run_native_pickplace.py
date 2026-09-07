#!/usr/bin/env python3
"""Fixed original gluestick regression, cuRobo only, no hardware or arbitrary argv."""
import argparse
import contextlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import traceback
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.workcell.pickplace_curobo_only import source_adapter,install
from rm75_app.workcell.native_outcome import NativeOutcomeCapture


FIXED_CASES = {
    'legacy_gluestick': ('rm75_app/_vendor/working_snapshot/pick_jiaobang/test_scenes/gluestick_desk_regression.json', ('gluestick',)),
    'current_table_all': ('assets/test_scenes/current_table.json',
        ('lvmukuai','carriot','shuazi','hongshupian','gluestick','bi','tennis')),
    'gluestick_jitter_00': ('assets/test_scenes/generated_random_batch_001/jitter_yaw_gluestick_desk_regression_01_00.json', ('gluestick',)),
    'gluestick_yaw_00': ('assets/test_scenes/generated_random_batch_001/yaw_only_gluestick_desk_regression_01_00.json', ('gluestick',)),
    'gluestick_swap_00': ('assets/test_scenes/generated_random_batch_001/same_class_swap_jitter_gluestick_desk_regression_01_00.json', ('gluestick',)),
}


def build_native_argv(case,extensions,*,transport_world_checked=False):
    path=ROOT/'rm75_app/_vendor/working_snapshot/pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    scene,names=FIXED_CASES[case]
    argv=[str(path),'--object-name',names[0],'--skip-foundationpose',
        '--fixed-scene-pose-file',str(ROOT/scene),'--render-mode','none','--auto-execute',
        '--curobo-torch-extensions-dir',str(Path(extensions).resolve())]
    if len(names)>1:argv+=['--cycle-object-names',*names]
    if transport_world_checked:
        # The existing world-only adapter cannot scope captured CUDA graphs.
        # Keep all original IK seeds/candidates, changing execution mode only.
        argv+=['--no-fast-chain-cuda-graph-ik']
    return argv


def main():
    started=time.monotonic()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extensions',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--audit-clearance',action='store_true')
    parser.add_argument('--audit-lift-ik',action='store_true',
        help='Read-only first failed still-attached lift IK per source; original solver results unchanged')
    parser.add_argument('--transport-world-checked',action='store_true',
        help='SIM only: reuse audited native contact compatibility, fully checking loaded transport')
    parser.add_argument('--case',choices=tuple(FIXED_CASES),default='legacy_gluestick')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    root=ROOT/'rm75_app/_vendor/working_snapshot'
    provenance=verify_snapshot(root)
    path=root/'pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    _,names=FIXED_CASES[args.case]
    argv=build_native_argv(args.case,args.extensions,transport_world_checked=args.transport_world_checked)
    report={'argv':argv,'execute_real':False,'command_success':False,'verified_task_success':None,
            'source_commit':provenance['source_commit'],'planner':'curobo_only',
            'case':args.case,'expected_cycles':len(names),
            'transport_world_checked_requested':args.transport_world_checked}
    contact_rows=[];cleanup=lambda:None
    captured=NativeOutcomeCapture(sys.stdout)
    sys.argv=argv;os.chdir(root);os.environ['LEROBOT_ROOT']=str(root)
    sys.path[:0]=[str(path.parent),str(root)]
    with source_adapter(root):
        try:
            spec=importlib.util.spec_from_file_location('_pickplace_native',path)
            module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module
            spec.loader.exec_module(module)
            report['clearance_path_audits']=install(module)
            if args.audit_lift_ik:
                from rm75_app.workcell.pickplace_lift_diagnostics import install_lift_diagnostics
                def emit_lift(row):
                    with (output/'lift_ik.jsonl').open('a') as stream:
                        stream.write(json.dumps(row,allow_nan=False)+'\n')
                report['lift_ik_diagnostics']=install_lift_diagnostics(module,emit_lift)
            if args.transport_world_checked:
                from curobo_rm75_planner import RM75CuRoboPlanner
                from rm75_app.workcell.transport_contact import install_transport_contact
                def emit_contact(row):
                    contact_rows.append(row)
                    with (output/'transport.jsonl').open('a') as stream:
                        stream.write(json.dumps(row)+'\n')
                cleanup=install_transport_contact(module,RM75CuRoboPlanner,emit_contact)
            if args.audit_clearance:
                from rm75_app.workcell.pickplace_clearance_audit import install as install_audit
                def emit(row):
                    with (output/'clearance.jsonl').open('a') as stream:
                        stream.write(json.dumps(row)+'\n')
                install_audit(module,emit)
            with contextlib.redirect_stdout(captured):
                value=module.main()
            if not report['clearance_path_audits']:
                raise RuntimeError('fixed regression produced no clearance execution audit')
            if not captured.report(len(names))['native_full_chain_passed']:
                raise RuntimeError('native cycle outcomes are failed or incomplete')
            report.update(command_success=value in (None,0),native_return=value,status='native_return_unverified')
        except BaseException as exc:
            report.update(status='failed',error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        finally:
            try:cleanup()
            except BaseException as exc:
                report.update(command_success=False,status='failed',cleanup_error=f'{type(exc).__name__}: {exc}')
            report['transport_path_audits']=[{k:row.get(k) for k in ('event','step_id','samples',
                'candidate_paths','payload_spheres','world_exempt_links','scene_fingerprint')}
                for row in contact_rows if row['event']=='transport_full_world_audit']
            report['transport_policy_rejections']=[row for row in contact_rows if row['event']=='transport_policy_rejected']
            report['native_cycles']=captured.cycles
            report['native_completed_cycles']=captured.report(len(names))['native_completed_cycles']
            report['diagnostic_rejections']=getattr(locals().get('module'),'_curobo_diagnostic_rejections',[])
            report['native_final_success']=captured.final
            report['clearance_failures']=captured.clearance_failures
            report['clearance_selection_audits']=getattr(locals().get('module'),'_clearance_selection_audits',[])
            report['strict_clearance_success']=(report['command_success'] and
                not captured.clearance_failures and
                len(report.get('clearance_path_audits',[]))==len(names))
            report['loaded_mplib_modules']=[n for n in sys.modules if n=='mplib' or n.startswith('mplib.')]
            report['elapsed_s']=time.monotonic()-started
            atomic_json(output/'result.json',report)
    print(json.dumps(report))
    return 0 if report['strict_clearance_success'] else 42


if __name__=='__main__':
    raise SystemExit(main())
