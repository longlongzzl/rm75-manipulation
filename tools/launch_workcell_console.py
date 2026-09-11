#!/usr/bin/env python3
"""Launch the unified preview/SIM console. Never enables real mode.

Without --profile, the shipped example opens an unconfigured preview/SIM starting point.
Use --list-profiles to locate existing machine configurations without selecting,
editing or merging them automatically. No hardware or GPU probe is performed.
"""
from __future__ import annotations
import argparse
import json
from itertools import islice
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import read_json


def profiles(root=ROOT):
    candidates=[root/'runtime_data/iteration/machine.json',root/'examples/workcell/machine.example.json']
    for pattern in ('runtime_data/three_scene/*/machine*.json','runtime_data/three_scene/*/*/machine*.json',
                    'configs/workcell/*machine*.json','runtime_data/iteration/*.json'):
        candidates.extend(islice(root.glob(pattern),max(0,500-len(candidates))))
    rows=[];seen=set()
    for path in candidates[:500]:
        if path in seen or not path.is_file():continue
        seen.add(path)
        try:value=read_json(path,max_bytes=2_000_000)
        except (OSError,ValueError):continue
        if not isinstance(value,dict) or value.get('schema')!='rm75_workcell_machine_v1':continue
        rows.append({'path':str(path),'pickplace_python':value.get('pickplace',{}).get('python'),
                     'has_jimu_library':bool(value.get('magnetic',{}).get('design_library')),
                     'physics_backends':value.get('pusht',{}).get('physics',{}).get('enabled_backends',[]),
                     'selected_automatically':False})
    return rows


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path)
    parser.add_argument('--port',type=int,default=7861)
    parser.add_argument('--list-profiles',action='store_true')
    parser.add_argument('--doctor',action='store_true',help='Read configuration readiness, without devices or execution')
    args=parser.parse_args(argv)
    if args.list_profiles:
        print(json.dumps({'profiles':profiles(),'hardware_contacted':False},ensure_ascii=False,indent=2));return 0
    if not 1024<=args.port<=65535:parser.error('port must be within 1024..65535')
    profile=(args.profile or ROOT/'examples/workcell/machine.example.json').expanduser().resolve()
    if not profile.is_file():parser.error('profile does not exist; use --list-profiles to inspect local candidates')
    value=read_json(profile)
    if not isinstance(value,dict) or value.get('schema')!='rm75_workcell_machine_v1':parser.error('not a workcell machine profile')
    if args.profile is None:
        print('使用示例配置：前端可以打开，不代表本机仿真或模型接口已就绪。',flush=True)
    print('统一控制台不开放真机启动；不会修改任何机器资格字段。',flush=True)
    if args.doctor:
        from rm75_app.workcell.service import WorkcellService
        from rm75_app.workcell.iteration_api import IterationAPI
        from rm75_app.workcell.console_api import ConsoleAPI
        service=WorkcellService(ROOT,profile,allow_real=False)
        result=ConsoleAPI(service,IterationAPI(service)).bootstrap()
        print(json.dumps({k:result[k] for k in ('version','profile_name','checks','mode_policy')},ensure_ascii=False,indent=2))
        return 0
    from rm75_app.workcell.cli import main as serve
    try:return serve(['serve','--profile',str(profile),'--port',str(args.port),'--app-root',str(ROOT)])
    except OSError as exc:
        print(f'Web服务启动失败：{exc}。未停止其它进程；可更换 --port。',file=sys.stderr);return 1

if __name__=='__main__':raise SystemExit(main())
