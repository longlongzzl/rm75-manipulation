#!/usr/bin/env python3
"""Fixed original gluestick regression, cuRobo only, no hardware or arbitrary argv."""
import argparse
import contextlib
import importlib.util
import json
import os
import re
from pathlib import Path
import sys
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.workcell.pickplace_curobo_only import source_adapter,install


FIXED_CASES = {
    'legacy_gluestick': ('rm75_app/_vendor/working_snapshot/pick_jiaobang/test_scenes/gluestick_desk_regression.json', ('gluestick',)),
    'current_table_all': ('assets/test_scenes/current_table.json',
        ('lvmukuai','carriot','shuazi','hongshupian','gluestick','bi','tennis')),
    'gluestick_jitter_00': ('assets/test_scenes/generated_random_batch_001/jitter_yaw_gluestick_desk_regression_01_00.json', ('gluestick',)),
    'gluestick_yaw_00': ('assets/test_scenes/generated_random_batch_001/yaw_only_gluestick_desk_regression_01_00.json', ('gluestick',)),
    'gluestick_swap_00': ('assets/test_scenes/generated_random_batch_001/same_class_swap_jitter_gluestick_desk_regression_01_00.json', ('gluestick',)),
}


class NativeOutcomeCapture:
    def __init__(self, stream):
        self.stream=stream;self.pending='';self.cycles=[];self.final=None;self.clearance_failures=[]
    def __getattr__(self, name):return getattr(self.stream,name)
    def write(self,text):
        self.stream.write(text);self.pending+=text
        while '\n' in self.pending:
            line,self.pending=self.pending.split('\n',1)
            match=re.fullmatch(r'cycle (\d+) success = (True|False)',line.strip())
            if match:self.cycles.append({'cycle':int(match[1]),'success':match[2]=='True'})
            match=re.fullmatch(r'final success = (True|False)',line.strip())
            if match:self.final=match[1]=='True'
            if line.startswith('[warn] post-place clearance planning failed after release;'):
                self.clearance_failures.append(line)
        return len(text)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extensions',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--audit-clearance',action='store_true')
    parser.add_argument('--case',choices=tuple(FIXED_CASES),default='legacy_gluestick')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    root=ROOT/'rm75_app/_vendor/working_snapshot'
    provenance=verify_snapshot(root)
    path=root/'pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    scene,names=FIXED_CASES[args.case]
    argv=[str(path),'--object-name',names[0],'--skip-foundationpose',
        '--fixed-scene-pose-file',str(ROOT/scene),
        '--render-mode','none','--auto-execute','--curobo-torch-extensions-dir',str(args.extensions.resolve())]
    if len(names)>1:argv+=['--cycle-object-names',*names]
    report={'argv':argv,'execute_real':False,'command_success':False,'verified_task_success':None,
            'source_commit':provenance['source_commit'],'planner':'curobo_only',
            'case':args.case,'expected_cycles':len(names)}
    captured=NativeOutcomeCapture(sys.stdout)
    sys.argv=argv;os.chdir(root);os.environ['LEROBOT_ROOT']=str(root)
    sys.path[:0]=[str(path.parent),str(root)]
    with source_adapter(root):
        try:
            spec=importlib.util.spec_from_file_location('_pickplace_native',path)
            module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module
            spec.loader.exec_module(module)
            report['clearance_path_audits']=install(module)
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
            if (captured.final is not True or len(captured.cycles)!=len(names)
                or not all(row['success'] for row in captured.cycles)):
                raise RuntimeError('native cycle outcomes are failed or incomplete')
            report.update(command_success=value in (None,0),native_return=value,status='native_return_unverified')
        except BaseException as exc:
            report.update(status='failed',error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        finally:
            report['native_cycles']=captured.cycles
            report['native_final_success']=captured.final
            report['clearance_failures']=captured.clearance_failures
            report['strict_clearance_success']=(report['command_success'] and
                not captured.clearance_failures and
                len(report.get('clearance_path_audits',[]))==len(names))
            report['loaded_mplib_modules']=[n for n in sys.modules if n=='mplib' or n.startswith('mplib.')]
            atomic_json(output/'result.json',report)
    print(json.dumps(report))
    return 0 if report['command_success'] else 42


if __name__=='__main__':
    raise SystemExit(main())
