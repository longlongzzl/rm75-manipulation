#!/usr/bin/env python3
"""Fixed original gluestick regression, cuRobo only, no hardware or arbitrary argv."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.workcell.pickplace_curobo_only import source_adapter,install


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extensions',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--audit-clearance',action='store_true')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    root=ROOT/'rm75_app/_vendor/working_snapshot'
    provenance=verify_snapshot(root)
    path=root/'pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    argv=[str(path),'--object-name','gluestick','--skip-foundationpose',
        '--fixed-scene-pose-file','pick_jiaobang/test_scenes/gluestick_desk_regression.json',
        '--render-mode','none','--auto-execute','--curobo-torch-extensions-dir',str(args.extensions.resolve())]
    report={'argv':argv,'execute_real':False,'command_success':False,'verified_task_success':None,
            'source_commit':provenance['source_commit'],'planner':'curobo_only'}
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
            value=module.main()
            if not report['clearance_path_audits']:
                raise RuntimeError('fixed gluestick regression produced no clearance execution audit')
            report.update(command_success=value in (None,0),native_return=value,status='native_return_unverified')
        except BaseException as exc:
            report.update(status='failed',error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        finally:
            report['loaded_mplib_modules']=[n for n in sys.modules if n=='mplib' or n.startswith('mplib.')]
            atomic_json(output/'result.json',report)
    print(json.dumps(report))
    return 0 if report['command_success'] else 42


if __name__=='__main__':
    raise SystemExit(main())
