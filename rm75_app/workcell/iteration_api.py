"""Authenticated UI integration for generation, random cases and boundary controls.

Mounted after the existing WSGI same-origin/token check. LLM requests run on a
separate bounded thread so an outstanding generation never blocks Stop.
"""
from __future__ import annotations
import concurrent.futures
import copy
from pathlib import Path
import re
import threading
import uuid

from .io import atomic_json, read_json, integer
from rm75_app.magnetic.generation import load_library, catalog_summary, generate
from rm75_app.magnetic.completion_provider import completion_client
from rm75_app.pusht.scenarios import sample_scenarios, configured_model, GEOMETRY_FIELDS
from rm75_app.pusht.model import valid_pose
from .transforms import vector

ID = re.compile(r'[0-9a-f]{32}\Z')
PREFIX = '/api/workcell/iterate/'


def _mapping(value, fields):
    if not isinstance(value, dict) or set(value)-set(fields):
        raise ValueError('Unexpected workflow request fields')
    return value


class IterationAPI:
    def __init__(self, service, *, completion=None):
        self.service=service; self.completion=completion
        self._lock=threading.RLock(); self._jobs={}; self._worker=None

    def features(self):
        errors=[]
        try:
            library=load_library(self.service.profile); catalog=catalog_summary(library)
        except (ValueError, OSError, KeyError, TypeError):
            catalog=[]; errors.append('Original grid/arc template library is not installed or failed validation')
        try:
            llm=completion_client(self.service.profile.get('magnetic',{}).get('llm',{})).readiness()
        except (ValueError, TypeError):
            llm={'configured':False,'missing':['valid server-side LLM settings']}
        variants=self.service.profile.get('pusht',{}).get('physics',{}).get('geometry_variants',{})
        geometries={key:{field:getattr(configured_model(self.service.profile,geometry_id=key),field)
                         for field in sorted(GEOMETRY_FIELDS)} for key in ['original',*sorted(variants)]}
        return dict(schema='rm75_iteration_features_v1',jimu_catalog=catalog,llm=llm,errors=errors,
            geometry_ids=['original',*sorted(variants)],geometry_models=geometries,
            sequence_native_sim_enabled=self.service.profile.get('pickplace',{}).get('fixed_scene_format')=='native_world',
            hardware_authorized=False, original_bases_are_not_synthesized=True)

    def _generation(self, payload):
        _mapping(payload, ('board_id','prompt','piece_budget'))
        if self.service.active:
            raise RuntimeError('Generate while the workcell is idle; generation never starts a robot task')
        library=load_library(self.service.profile)
        request=copy.deepcopy(payload)
        # Check cheap fields before scheduling/paying for a model call.
        if request.get('board_id') not in library['boards']:
            raise ValueError('Original base is not installed')
        integer(request.get('piece_budget',12),'piece_budget',1,12)
        if not isinstance(request.get('prompt'),str) or not 1<=len(request['prompt'].strip())<=1500:
            raise ValueError('Describe the structure in 1..1500 characters')
        complete=self.completion or completion_client(self.service.profile.get('magnetic',{}).get('llm',{}))
        if self.completion is None and not complete.readiness()['configured']:
            raise ValueError('Configure the server LLM before generating a design')
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError('One bounded design generation is already running')
            # Keep at most 32 completed jobs in memory; persistent JSON has no key/URL.
            while len(self._jobs)>=32:
                self._jobs.pop(next(iter(self._jobs)))
            ident=uuid.uuid4().hex
            directory=self.service.root/'design_generations'/ident
            directory.mkdir(parents=True,exist_ok=False)
            self._jobs[ident]={'generation_id':ident,'status':'running'}
            atomic_json(directory/'request.json',request)
            def work():
                try:
                    result=generate(library,request,complete)
                    final=dict(generation_id=ident,status='succeeded',**result,
                               robot_command_submitted=False)
                except Exception as exc:
                    # No endpoint URLs, tokens, prompts or response body in an error.
                    final=dict(generation_id=ident,status='failed',error_type=type(exc).__name__,
                               error='No valid original-template design accepted; inspect server configuration or revise the request',
                               robot_command_submitted=False)
                with self._lock:
                    self._jobs[ident]=final
                    atomic_json(directory/'result.json',final)
            self._worker=threading.Thread(target=work,name='jimu-design-generation',daemon=True)
            self._worker.start()
            return dict(self._jobs[ident])

    def generation(self, ident):
        if not ID.fullmatch(ident): raise ValueError('Invalid generation id')
        with self._lock:
            if ident in self._jobs: return copy.deepcopy(self._jobs[ident])
        path=self.service.root/'design_generations'/ident/'result.json'
        if not path.is_file(): raise FileNotFoundError('Unknown generation')
        return read_json(path,max_bytes=2_000_000)

    def _session(self, ident, *, active=False):
        if not ID.fullmatch(ident): raise ValueError('Invalid session id')
        root=self.service.root/'jobs'/ident
        request=read_json(root/'request.json',max_bytes=2_000_000)
        if request.get('task')!='pusht' or request.get('mode')!='sim':
            raise PermissionError('Interactive controls are currently qualified for SIM only, not hardware')
        if 'run_until_goal' not in request.get('parameters',{}):
            raise ValueError('This request did not enable the interactive session contract')
        if active and (self.service.active!=ident or (root/'result.json').exists()):
            raise ValueError('Interactive session is no longer running')
        return root, request

    def session(self, ident):
        root,_=self._session(ident)
        try: result=read_json(root/'session_status.json',max_bytes=8192)
        except FileNotFoundError: result={'phase':'starting','safe_to_adjust':False}
        if (root/'result.json').exists():
            result={**result,'phase':'finished','safe_to_adjust':False,'finished':True}
        return result

    def control(self, ident, payload):
        _mapping(payload,('action','pose'))
        action=payload.get('action')
        if action not in ('pause','resume','relocate'):
            raise ValueError('Only pause, resume and explicit simulated relocation are supported')
        if set(payload)!=( {'action','pose'} if action=='relocate' else {'action'} ):
            raise ValueError('Unexpected control data')
        with self.service._lock:
            root,request=self._session(ident,active=True)
            state=self.session(ident)
            if action=='resume' and state.get('phase') not in ('paused','waiting_for_scene_change'):
                raise ValueError('Wait for the acknowledged boundary pause before resuming')
            if action=='relocate':
                if (state.get('phase')!='paused' or state.get('safe_to_adjust') is not True
                        or request['parameters'].get('simulation_backend') not in ('tool_only_physics','full_arm_physics')):
                    raise PermissionError('Relocation requires an acknowledged paused physics session')
                pose=vector(payload['pose'],3,'pose').tolist()
                profile=read_json(root/'machine_profile.json')
                cfg=configured_model(profile,geometry_id=request['parameters'].get('geometry_id','original'),
                                     maximum_push_length_m=request['parameters'].get('maximum_push_length_m'))
                if not valid_pose(pose,cfg): raise ValueError('T relocation crosses workspace or obstacle')
                payload={'action':action,'pose':pose}
            directory=root/'session_commands'; directory.mkdir(exist_ok=True)
            numbers=[int(p.stem) for p in directory.glob('*.json') if re.fullmatch(r'\d{8}',p.stem)]
            last=max(numbers,default=0)
            if last>=10000: raise RuntimeError('Session control budget exhausted')
            if last>state.get('last_command',0):
                raise RuntimeError('Wait for the previous session command acknowledgement')
            sequence=last+1
            atomic_json(directory/f'{sequence:08d}.json',dict(sequence=sequence,**payload))
            return dict(queued=True,sequence=sequence,acknowledged=False,
                        safe_to_adjust=False,stop_is_separate=True)

    def get(self, suffix):
        parts=suffix.strip('/').split('/')
        if parts==['features']: return self.features()
        if len(parts)==2 and parts[0]=='generations': return self.generation(parts[1])
        if len(parts)==2 and parts[0]=='sessions': return self.session(parts[1])
        raise FileNotFoundError('Unknown iteration route')

    def post(self, suffix, payload):
        parts=suffix.strip('/').split('/')
        if parts==['generate']: return self._generation(payload)
        if parts==['randomize']:
            _mapping(payload,('seed','count','geometry_id'))
            return sample_scenarios(self.service.profile,**payload)
        if len(parts)==3 and parts[0]=='sessions' and parts[2]=='control':
            return self.control(parts[1],payload)
        raise FileNotFoundError('Unknown iteration route')
