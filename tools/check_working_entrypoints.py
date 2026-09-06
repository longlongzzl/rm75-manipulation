#!/usr/bin/env python3
"""Inspect the migrated native parser using its actual Python environment.

Loads original modules and constructs argparse only; never calls original main(),
MotionGen, a camera capture or the real execution adapter explicitly. The native
imports still require their original cuRobo/ManiSkill/vision dependencies.
"""
from __future__ import annotations
import argparse
import os
import subprocess
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json,read_json,dumps
from rm75_app.workcell.legacy import import_working_entry,original_parser,snapshot_root,build_native_argv
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.workcell.spec import validate_spec


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--task',choices=('pickplace','magnetic'),required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--in-native-python',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--request',type=Path)
    a=p.parse_args();a.profile=a.profile.resolve();a.output=a.output.resolve()
    if a.request:
        a.request=a.request.resolve()
    profile=read_json(a.profile)
    if not a.in_native_python:
        interpreter=profile[a.task]['python']
        if not interpreter or not Path(interpreter).is_file():
            raise FileNotFoundError(f'{a.task}: configure the actual working Python environment first')
        command=[interpreter,str(Path(__file__).resolve()),'--profile',str(a.profile.resolve()),
                 '--task',a.task,'--output',str(a.output.resolve()),'--in-native-python']
        if a.request:
            command+=['--request',str(a.request.resolve())]
        env=dict(os.environ);env['PYTHONPATH']=str(ROOT)+os.pathsep+env.get('PYTHONPATH','')
        return subprocess.run(command,cwd=ROOT,env=env,check=False).returncode
    snapshot=snapshot_root(ROOT);provenance=verify_snapshot(snapshot)
    os.chdir(snapshot)
    module=import_working_entry(snapshot,a.task)
    parser=original_parser(module,a.task)
    options=parser._option_string_actions
    required=['--auto-execute','--execute-real']+(['--object-name'] if a.task=='pickplace' else ['--jimu-builder-scene-json','--jimu-apriltag-anchor-localization'])
    missing=[name for name in required if name not in options]
    report={'task':a.task,'source_commit':provenance['source_commit'],'python':sys.executable,
            'native_main_called':False,'planner_called':False,'robot_control_called':False,
            'missing_required_flags':missing,'option_destinations':{name:action.dest for name,action in options.items()},
            'fixed_scene_candidates':[x['path'] for x in provenance['files'] if x['path'].endswith('.json')
                                      and '/scenes/' in x['path'] and 'jimu' in x['path'].lower()]}
    if a.request:
        spec=validate_spec(read_json(a.request),profile)
        if spec['task']!=a.task or spec['mode']=='real':
            raise ValueError('Contract check accepts matching preview/sim requests only')
        spec['mode']='sim'
        a.output.resolve().parent.mkdir(parents=True,exist_ok=True)
        report['parsed_native_argv']=build_native_argv(module,spec,profile,a.output.resolve().parent,snapshot)
    atomic_json(a.output.resolve(),report);print(dumps(report))
    return int(bool(missing))


if __name__=='__main__':
    raise SystemExit(main())
