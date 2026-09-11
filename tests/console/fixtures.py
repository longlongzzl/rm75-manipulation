"""UI fixture only: no robot, SDK, solver, native worker or model API calls.

The real ConsoleAPI and WSGI are exercised around these explicit substitutes.
Not imported by the production console or launcher.
"""
import copy
import hashlib
import sys
import threading
import time
import uuid
from pathlib import Path
from types import ModuleType
from rm75_app.workcell.io import read_json, atomic_json, digest
from rm75_app.magnetic.design import validate_design, piece_key


def design(board='grid_3x3'):
    pieces=[]
    for i in range(9 if board=='grid_3x3' else 7):
        pieces.append(dict(role=f'fixed_{i}',id=f'fixed_{i}',type='square',locked=True,
            center=[(i%3-1)*.074,.00325,(i//3-1)*.074],u=[1,0,0],n=[0,1,0],v=[0,0,1]))
    for i in range(3):
        pieces.append(dict(role=f'wall_{i}',id=f'wall_{i}',type='triangle' if i==2 else 'square',locked=False,
            center=[(i-1)*.08,.0435 if i<2 else .0675,.1],u=[1,0,0],n=[0,0,-1],v=[0,1,0],parentRole=f'fixed_{i}'))
    return {'schema':'jimu_builder_scene_v1','pieces':pieces}


def library():
    return {'boards':{b:{'templates':[{'id':b+'_00','title':'UI fixture / '+b,'design':design(b),
        'design_digest':digest(design(b))}]} for b in ('grid_3x3','arc')}}


def compile_selection(lib,proposal,board_id,**kw):
    d=lib['boards'][board_id]['templates'][0]['design']
    proof={**proposal,'design_digest':digest(d),'movable_count':3,'locked_digest':digest([p for p in d['pieces'] if p['locked']])}
    return {'design':copy.deepcopy(d),'proof':proof}


def stub_validate(value,profile):
    if not isinstance(value,dict) or value.get('task') not in ('pickplace','magnetic','pusht'):
        raise ValueError('Unknown task')
    if value.get('mode') not in ('preview','sim','real'):raise ValueError('Unknown mode')
    p=value.get('parameters',{})
    if value['task']=='pickplace':
        requested=p.get('object_names',[p.get('object_name')])
        if not requested or any(n not in profile['pickplace']['object_names'] for n in requested):raise ValueError('Unknown object')
    if value['task']=='magnetic':validate_design(p['design'])
    return copy.deepcopy(value)


class FakeService:
    def __init__(self,root):
        self.app_root=Path(root);self.root=self.app_root/'runtime_data/workcell';self.root.mkdir(parents=True)
        self.profile_path=self.app_root/'UI_FIXTURE_ONLY.json';self.csrf='fixture-token';self._lock=threading.RLock()
        self.active=None;self.latch=self.root/'REAL_REVIEW_REQUIRED.json';self.allow_real=False
        self.submissions=[];self.stops=[];self.inputs=[];self.sessions={};self.delay=.08
        scene=self.app_root/'fixture_scene.json';atomic_json(scene,{'fixture_only':True})
        marker=self.app_root/'rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json'
        atomic_json(marker,{'fixture_only':True})
        lib=self.app_root/'fixture_library.json';atomic_json(lib,library())
        self.profile={'schema':'rm75_workcell_machine_v1','hardware':{'hardware_reviewed':False},
            'pickplace':{'python':sys.executable,'fixed_scene':str(scene),'fixed_scene_format':'native_world',
                'object_names':['shuazi','bi','lvmukuai','carriot','tennis','gluestick','hongshupian']},
            'magnetic':{'python':sys.executable,'fixed_scene':str(scene),'design_library':str(lib),'llm':{'timeout_s':300}},
            'pusht':{'model':{'workspace':[.15,.65,-.3,.3],'bar_width_m':.1,'bar_height_m':.03,'stem_width_m':.03,'stem_height_m':.07},
                'physics':{'enabled_backends':['full_arm_physics'],'simulation_python':sys.executable,
                'planner_python':sys.executable,'motion':{'fixture_only':True}}}}
        atomic_json(self.profile_path,self.profile)
    def info(self):
        return dict(csrf=self.csrf,allow_real=False,real_latched=False,active_job=self.active,
            pickplace_objects=self.profile['pickplace']['object_names'],pusht_model=self.profile['pusht']['model'],
            pusht_simulation_backends=['surrogate','full_arm_physics'],pusht_physics_initial_pose=[.35,-.18,0],snapshot_installed=True)
    def submit(self,spec,arm_token=None):
        assert spec['mode']!='real' and arm_token is None
        if self.active:raise RuntimeError('Busy fixture')
        ident=uuid.uuid4().hex;root=self.root/'jobs'/ident;root.mkdir(parents=True)
        self.submissions.append(copy.deepcopy(spec));atomic_json(root/'request.json',spec);atomic_json(root/'machine_profile.json',self.profile)
        self.active=ident
        if spec['task']=='pusht':self.sessions[ident]=dict(phase='observing',safe_to_adjust=False,step=1,epoch=0,last_command=0,last_pose=[.35,-.18,0])
        def finish():
            time.sleep(self.delay)
            if self.active==ident and spec['mode']=='preview':self.complete(ident)
        threading.Thread(target=finish,daemon=True).start()
        return {'job_id':ident,'status':'running'}
    def complete(self,ident):
        req=read_json(self.root/'jobs'/ident/'request.json')
        r={'status':'command_completed_unverified','task_success':None,'command_success':True,
           'verification':'preview_only' if req['mode']=='preview' else 'ui_fixture_only',
           'note':'UI substitute only; no native worker, GPU or physics has run'}
        atomic_json(self.root/'jobs'/ident/'result.json',r)
        if self.active==ident:self.active=None
    def job(self,ident):
        root=self.root/'jobs'/ident
        if not root.is_dir():raise FileNotFoundError('Unknown fixture job')
        result=read_json(root/'result.json') if (root/'result.json').is_file() else None
        return {'job_id':ident,'request':read_json(root/'request.json'),'status':(result or {}).get('status','running'),
            **({'result':result} if result else {}),'log':'UI FIXTURE ONLY — no robot, native solver or physical execution.\n',
            'events':[{'kind':'task_started','at':time.time()}]}
    def cancel(self,ident):
        self.stops.append(ident);atomic_json(self.root/'jobs'/ident/'result.json',{'status':'cancelled','task_success':None,'command_success':False})
        self.active=None;return {'requested':True,'physical_estop':False}
    def respond_input(self,ident,nonce,value):self.inputs.append((ident,nonce,value));return {'accepted':True}


class FakeIteration:
    def __init__(self,service):self.service=service;self.calls=[];self._jobs={};self._workers={};self.delay=.15;self.controls=[]
    def features(self):
        catalog=[]
        for b,title in [('grid_3x3','UI示例 · 3×3'),('arc','UI示例 · 弧形')]:
            catalog.append(dict(board_id=b,board_title=title,template_id=b+'_00',title='测试结构（非原底板数据）'))
        return {'jimu_catalog':catalog,'llm':{'configured':True,'provider':'UI_FIXTURE_NO_NETWORK'},
            'geometry_ids':['original'],'geometry_models':{'original':self.service.profile['pusht']['model']},
            'errors':[],'sequence_native_sim_enabled':True}
    def post(self,suffix,payload):
        if suffix=='generate':
            if self.service.active:raise RuntimeError('Busy')
            if any(x.get('status')=='running' for x in self._jobs.values()):raise RuntimeError('Already generating')
            self.calls.append(copy.deepcopy(payload));ident=uuid.uuid4().hex
            root=self.service.root/'design_generations'/ident;root.mkdir(parents=True);atomic_json(root/'request.json',payload)
            self._jobs[ident]={'status':'running','generation_id':ident}
            def finish():
                time.sleep(self.delay);self.finish(ident,payload)
            thread=threading.Thread(target=finish,daemon=True);thread.start();self._workers[ident]=thread;self._worker=thread
            return self._jobs[ident]
        if suffix=='randomize':
            return {'cases':[{'case_id':f"{payload['seed']}_{i}",'initial_pose':[.34+i*.01,-.17,.1],
                'goal_pose':[.38,-.18,0],'mode':['translation','rotation','mixed'][i]} for i in range(3)]}
        if suffix.endswith('/control'):return self.control(suffix.split('/')[1],payload)
        raise FileNotFoundError(suffix)
    def finish(self,ident,payload):
        proposal=dict(board_id=payload['board_id'],template_id=payload['board_id']+'_00',title='UI fixture 三块结构',
            selected_roles=['wall_0','wall_1','wall_2'],explanation='演示异步生成交互；未调用模型，不是物理结果。')
        row={'generation_id':ident,'status':'succeeded',**compile_selection(library(),proposal,board_id=proposal['board_id'])}
        self._jobs[ident]=row;atomic_json(self.service.root/'design_generations'/ident/'result.json',row)
    def generation(self,ident):
        if ident in self._jobs:return self._jobs[ident]
        return read_json(self.service.root/'design_generations'/ident/'result.json')
    def get(self,suffix):
        if suffix.startswith('sessions/'):return self.service.sessions[suffix.split('/')[1]]
        if suffix.startswith('generations/'):return self.generation(suffix.split('/')[1])
        if suffix=='features':return self.features()
        raise FileNotFoundError(suffix)
    def control(self,ident,payload):
        assert self.service.active==ident
        row=self.service.sessions[ident];action=payload['action'];self.controls.append(action)
        if action=='pause':row.update(phase='paused',safe_to_adjust=True)
        elif action=='relocate':
            if not row['safe_to_adjust']:raise PermissionError('Not paused')
            row['last_pose']=payload['pose']
        elif action=='resume':row.update(phase='observing',safe_to_adjust=False)
        row['last_command']+=1;row['epoch']+=1
        return {'sequence':row['last_command'],'queued':True}


def install_substitutes(monkeypatch):
    m=ModuleType('rm75_app.workcell.iteration_api');m.IterationAPI=FakeIteration
    monkeypatch.setitem(sys.modules,m.__name__,m)
    m=ModuleType('rm75_app.workcell.spec');m.validate_spec=stub_validate
    monkeypatch.setitem(sys.modules,m.__name__,m)
    m=ModuleType('rm75_app.magnetic.generation');m.load_library=lambda profile:{**library(),'_fixture_fixed':profile['magnetic']['fixed_scene']};m.compile_selection=compile_selection
    def recipe(lib,design,proof):
        if digest(design)!=proof.get('design_digest'):raise ValueError('Changed design digest')
        path=Path(lib['_fixture_fixed'])
        return {'fixed_scene':str(path),'fixed_scene_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    m.validate_generated_request=recipe
    monkeypatch.setitem(sys.modules,m.__name__,m)
