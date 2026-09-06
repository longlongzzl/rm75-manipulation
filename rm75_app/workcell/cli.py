"""python -m rm75_app.workcell serve|run|doctor"""
from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path
from socketserver import ThreadingMixIn
from wsgiref.simple_server import WSGIServer,make_server
from .io import read_json,dumps


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['serve','run','doctor'])
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--app-root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--port',type=int,default=7861)
    parser.add_argument('--allow-real',action='store_true')
    parser.add_argument('--request',type=Path)
    parser.add_argument('--task',choices=('pickplace','magnetic','pusht'))
    parser.add_argument('--mode',choices=('preview','sim','real'))
    args=parser.parse_args(argv)
    if args.command=='doctor':
        from .migration import verify_snapshot
        from .legacy import snapshot_root
        profile=read_json(args.profile)
        report={'profile_schema':profile.get('schema'),'python':sys.executable,'robot_contacted':False,'issues':[]}
        try:
            source=verify_snapshot(snapshot_root(args.app_root))
            report['source_commit']=source['source_commit'];report['migrated_files']=source['file_count']
        except Exception as exc:
            report['issues'].append(str(exc))
        for task in ('pickplace','magnetic','pusht'):
            interpreter=profile.get(task,{}).get('python',sys.executable)
            if not isinstance(interpreter,str) or not Path(interpreter).is_file():
                report['issues'].append(f'{task}: missing Python {interpreter}')
        print(dumps(report));return int(bool(report['issues']))
    from .service import WorkcellService
    service=WorkcellService(args.app_root,args.profile,allow_real=args.allow_real)
    try:
        if args.command=='run':
            if args.request is None:
                parser.error('--request is required')
            spec=read_json(args.request);token=None
            if args.task is not None:
                if spec.get('task')!=args.task:
                    raise ValueError('Task adapter and request JSON refer to different tasks')
            if args.mode is not None:
                spec['mode']=args.mode
            if spec.get('mode')=='real':
                print('输入：我确认现场安全并允许本次真机运行')
                token=service.arm(spec,input().strip())['arm_token']
            job=service.submit(spec,token)['job_id']
            answered=set()
            while service.active:
                state=service.job(job)
                prompt=state.get('input_request')
                if prompt and prompt['nonce'] not in answered:
                    print(prompt.get('prompt','Original runtime confirmation'),flush=True)
                    answer=input('[Enter / r / q] ').strip()
                    service.respond_input(job,prompt['nonce'],answer)
                    answered.add(prompt['nonce'])
                time.sleep(.1)
            result=service.job(job);print(dumps(result))
            return int(result['status'] in ('failed','cancelled','verification_failed'))
        from .server import WorkcellWSGI
        class ThreadingServer(ThreadingMixIn,WSGIServer):
            daemon_threads=True
        with make_server('127.0.0.1',args.port,WorkcellWSGI(service),server_class=ThreadingServer) as httpd:
            print(f'Workcell: http://127.0.0.1:{args.port}/workcell/ — real={args.allow_real}',flush=True)
            httpd.serve_forever()
    except KeyboardInterrupt:
        return 130
    finally:
        service.close()
    return 0
