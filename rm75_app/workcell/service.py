"""Single-job service: subprocess isolation, request-bound real authorization, stop.

The browser never passes shell commands. Cooperative STOP is paired with an
independent controller stop for real jobs. Stop uncertainty latches real access.
"""
from __future__ import annotations
import os
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from .io import atomic_json,read_json,digest,dumps,loads
from .spec import validate_spec
from .events import EventLog


def controller_stop(hardware):
    """Bounded JSON controlled-stop exchange; not a safety-rated physical E-stop."""
    with socket.create_connection((hardware['ip'],int(hardware.get('port',8080))),timeout=1.5) as sock:
        sock.settimeout(1.5)
        sock.sendall(b'{"command":"set_arm_stop"}\r\n')
        raw=b''
        while len(raw)<65536:
            data=sock.recv(4096)
            if not data:
                break
            raw+=data
            for line in raw.splitlines():
                try:
                    response=loads(line.decode())
                except (UnicodeError,ValueError):
                    continue
                if response.get('command')=='set_arm_stop':
                    if response.get('arm_stop') is not True:
                        raise RuntimeError(f'Controller refused stop: {response}')
                    return response
        raise TimeoutError('No controller stop acknowledgement')


class WorkcellService:
    def __init__(self,app_root,profile_path,*,allow_real=False,legacy_busy=None):
        self.app_root=Path(app_root).resolve();self.profile_path=Path(profile_path).resolve()
        self.profile=read_json(self.profile_path)
        if self.profile.get('schema')!='rm75_workcell_machine_v1':
            raise ValueError('Unsupported machine profile schema')
        self.root=self.app_root/'runtime_data'/'workcell'
        self.root.mkdir(parents=True,exist_ok=True)
        self.allow_real=allow_real;self.legacy_busy=legacy_busy or (lambda:False)
        self.csrf=secrets.token_urlsafe(32);self._lock=threading.RLock()
        self.active=None;self.process=None;self._tokens={};self._thread=None;self._input_nonces=set()
        # A previous real worker/process interruption is not silently cleared.
        self.latch=self.root/'REAL_REVIEW_REQUIRED.json'

    def info(self):
        return {'csrf':self.csrf,'allow_real':self.allow_real,
                'real_latched':self.latch.exists(),'active_job':self.active,
                'tasks':['pickplace','magnetic','pusht'],
                'pickplace_objects':self.profile.get('pickplace',{}).get('object_names',
                     ['lvmukuai','carriot','shuazi','gluestick','bi','tennis']),
                'pusht_model':self.profile.get('pusht',{}).get('model',{}),
                'snapshot_installed':(self.app_root/'rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json').is_file(),
                'physical_robot_tested_here':False}

    def arm(self,spec,confirmation):
        spec=validate_spec(spec,self.profile)
        with self._lock:
            if spec['mode']!='real' or not self.allow_real or self.latch.exists():
                raise PermissionError('Real mode disabled or requires local review')
            hardware=self.profile.get('hardware',{})
            if hardware.get('hardware_reviewed') is not True or not hardware.get('qualification_id'):
                raise PermissionError('Machine profile not qualified')
            if self.profile.get(spec['task'],{}).get('integration_qualified') is not True:
                raise PermissionError('This task integration has not passed local qualification')
            if confirmation!='我确认现场安全并允许本次真机运行':
                raise PermissionError('Explicit local operator confirmation is required')
            if self.active or self.legacy_busy():
                raise RuntimeError('A task already owns the workcell')
            self._tokens={k:v for k,v in self._tokens.items() if v[1]>time.monotonic()}
            token=secrets.token_urlsafe(32)
            self._tokens[token]=(digest(spec),time.monotonic()+60)
            return {'arm_token':token,'expires_in_s':60,'request_digest':digest(spec)}

    def submit(self,spec,arm_token=None):
        spec=validate_spec(spec,self.profile)
        with self._lock:
            if self.active or self.legacy_busy():
                raise RuntimeError('Another task is running; three tasks share one robot')
            authorized=False
            if spec['mode']=='real':
                entry=self._tokens.pop(arm_token,None)
                if not self.allow_real or self.latch.exists() or not entry or entry[1]<time.monotonic() or entry[0]!=digest(spec):
                    raise PermissionError('Missing, expired, consumed or mismatched real authorization')
                authorized=True
            job_id=uuid.uuid4().hex;directory=self.root/'jobs'/job_id
            directory.mkdir(parents=True)
            atomic_json(directory/'request.json',spec)
            # Snapshot server-side config so it cannot be changed after approval.
            atomic_json(directory/'machine_profile.json',self.profile)
            python=(sys.executable if spec['mode']=='preview' or spec['task']=='pusht' and spec['mode']=='sim'
                    else self.profile.get(spec['task'],{}).get('python',sys.executable))
            if not isinstance(python,str) or not Path(python).expanduser().is_file():
                raise FileNotFoundError(f'Configured Python interpreter is missing: {python}')
            command=[str(python),'-u','-m','rm75_app.workcell.worker','--run-dir',str(directory),
                     '--profile',str(directory/'machine_profile.json'),'--app-root',str(self.app_root)]
            if authorized:
                command.append('--real-authorized')
                atomic_json(self.latch,{'job_id':job_id,'reason':'real_job_in_progress','created_at':time.time()})
            env=dict(os.environ)
            env['PYTHONPATH']=str(self.app_root)+os.pathsep+env.get('PYTHONPATH','')
            try:
                with (directory/'stdout.log').open('wb') as output:
                    process=subprocess.Popen(command,cwd=self.app_root,env=env,stdout=output,
                                             stderr=subprocess.STDOUT,stdin=subprocess.PIPE,start_new_session=True)
            except BaseException:
                if authorized:
                    atomic_json(self.latch,{'job_id':job_id,'reason':'launch_failed_no_motion_assumed_unknown'})
                raise
            self.active=job_id;self.process=process
            self._thread=threading.Thread(target=self._wait,args=(job_id,process,spec),daemon=True)
            self._thread.start()
            return {'job_id':job_id,'status':'running','request_digest':digest(spec)}

    def _wait(self,job_id,process,spec):
        code=process.wait()
        directory=self.root/'jobs'/job_id
        with self._lock:
            if not (directory/'result.json').is_file():
                atomic_json(directory/'result.json',{'status':'cancelled' if (directory/'STOP').exists() else 'failed',
                    'command_success':False,'task_success':None,'error':f'worker exited {code} without a final result',
                    'mode':spec['mode'],'task':spec['task'],'finished_at':time.time()})
            result=read_json(directory/'result.json')
            if spec['mode']=='real':
                # Even nominal completion with unknown object outcome needs local
                # inspection before a different task can move the same arm.
                if result.get('status')=='succeeded' and result.get('task_success') is True:
                    self.latch.unlink(missing_ok=True)
                else:
                    atomic_json(self.latch,{'job_id':job_id,'reason':'real_run_requires_inspection','result':result})
            if self.active==job_id:
                self.active=None;self.process=None

    def job(self,job_id):
        if not isinstance(job_id,str) or len(job_id)!=32 or any(ch not in '0123456789abcdef' for ch in job_id):
            raise ValueError('Invalid job id')
        directory=self.root/'jobs'/job_id
        if not directory.is_dir():
            raise FileNotFoundError('Unknown job')
        result={'job_id':job_id,'status':'running','request':read_json(directory/'request.json')}
        for key,name in [('progress','progress.json'),('result','result.json'),('input_request','pending_input.json')]:
            try:
                result[key]=read_json(directory/name)
            except FileNotFoundError:
                pass
        if 'result' in result:
            result['status']=result['result']['status']
        event_file=directory/'events.jsonl'
        if event_file.exists():
            with event_file.open('rb') as stream:
                stream.seek(max(0,event_file.stat().st_size-120000))
                rows=stream.read().splitlines()
            events=[]
            for raw in rows[-40:]:
                try:
