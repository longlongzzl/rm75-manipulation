import io
import json
import threading
from pathlib import Path
from types import SimpleNamespace
import pytest
from rm75_app.workcell.io import atomic_json,read_json
from rm75_app.workcell.iteration_api import IterationAPI
from rm75_app.workcell.server import WorkcellWSGI


class Service:
    def __init__(self,tmp_path,profile):
        self.root=tmp_path/'workcell';self.root.mkdir();self.profile=profile
        self._lock=threading.RLock();self.active=None;self.csrf='test-token'
    def info(self):return {'csrf':self.csrf}


def session(service,*,mode='sim',backend='full_arm_physics',phase='paused'):
    ident='a'*32;root=service.root/'jobs'/ident;root.mkdir(parents=True)
    atomic_json(root/'request.json',{'task':'pusht','mode':mode,'parameters':{
        'run_until_goal':True,'simulation_backend':backend,'geometry_id':'original'}})
    atomic_json(root/'machine_profile.json',service.profile)
    atomic_json(root/'session_status.json',dict(phase=phase,safe_to_adjust=phase=='paused',last_command=0))
    service.active=ident
    return ident,root


def wsgi(app,path,body=None,token='test-token',host='127.0.0.1:7861',origin=None):
    raw=json.dumps(body).encode() if body is not None else b'';headers=[]
    env={'PATH_INFO':path,'REQUEST_METHOD':'POST' if body is not None else 'GET','HTTP_HOST':host,
         'HTTP_X_WORKCELL_TOKEN':token,'CONTENT_LENGTH':str(len(raw)),'wsgi.input':io.BytesIO(raw)}
    if origin:env['HTTP_ORIGIN']=origin
    data=b''.join(app(env,lambda status,items:headers.append((status,items))))
    return headers[0][0],data


def test_async_generate_compiles_no_motion(tmp_path,profile,proposal):
    service=Service(tmp_path,profile);calls=[]
    api=IterationAPI(service,completion=lambda messages:calls.append(messages) or json.dumps(proposal))
    reply=api.post('generate',{'board_id':'grid_3x3','prompt':'a small house'})
    api._worker.join(timeout=2);result=api.generation(reply['generation_id'])
    assert result['status']=='succeeded' and not result['robot_command_submitted']
    assert result['proof']['movable_count']==3 and len(calls)==1 and service.active is None


def test_generation_single_worker_backpressure(tmp_path,profile,proposal):
    gate=threading.Event()
    def complete(messages):gate.wait(2);return json.dumps(proposal)
    api=IterationAPI(Service(tmp_path,profile),completion=complete)
    api.post('generate',{'board_id':'arc','prompt':'x'})
    with pytest.raises(RuntimeError):api.post('generate',{'board_id':'arc','prompt':'x'})
    gate.set();api._worker.join(timeout=3)


@pytest.mark.parametrize('token,host,origin',[('bad','127.0.0.1:7861',None),('test-token','remote:7861',None),('test-token','127.0.0.1:7861','https://evil.example')])
def test_new_post_respects_original_auth(tmp_path,profile,token,host,origin):
    app=WorkcellWSGI(Service(tmp_path,profile))
    code,_=wsgi(app,'/api/workcell/iterate/randomize',{'seed':42,'count':1},token,host,origin)
    assert code.startswith('403')


def test_new_static_and_random_api(tmp_path,profile):
    app=WorkcellWSGI(Service(tmp_path,profile))
    code,body=wsgi(app,'/workcell/')
    assert code.startswith('200') and b'/workcell/iteration.js' in body and b'/workcell/app.js' in body
    for path in ('iteration.js','iteration.css'):
        code,body=wsgi(app,'/workcell/'+path);assert code.startswith('200') and body
    code,body=wsgi(app,'/api/workcell/iterate/randomize',{'seed':42,'count':6})
    assert code.startswith('200') and len(json.loads(body)['cases'])==6
    _,body=wsgi(app,'/api/workcell/iterate/features')
    assert 'wide' in json.loads(body)['geometry_ids']


def test_command_queue_ack_and_no_overwrite(tmp_path,profile):
    service=Service(tmp_path,profile);ident,root=session(service);api=IterationAPI(service)
    result=api.control(ident,{'action':'relocate','pose':[.36,0,0]})
    assert result['queued'] and not result['acknowledged'] and not result['safe_to_adjust']
    first=read_json(root/'session_commands/00000001.json')
    with pytest.raises(RuntimeError):api.control(ident,{'action':'resume'})
    assert read_json(root/'session_commands/00000001.json')==first
    atomic_json(root/'session_status.json',dict(phase='paused',safe_to_adjust=True,last_command=1))
    assert api.control(ident,{'action':'resume'})['sequence']==2


@pytest.mark.parametrize('phase,backend,mode',[('executing','full_arm_physics','sim'),('paused','surrogate','sim'),('paused','full_arm_physics','real')])
def test_relocation_never_during_motion_or_real(tmp_path,profile,phase,backend,mode):
    service=Service(tmp_path,profile);ident,_=session(service,phase=phase,backend=backend,mode=mode)
    with pytest.raises(PermissionError):IterationAPI(service).control(ident,{'action':'relocate','pose':[.36,0,0]})


def test_invalid_pose_and_finished_session(tmp_path,profile):
    service=Service(tmp_path,profile);ident,root=session(service);api=IterationAPI(service)
    with pytest.raises(ValueError):api.control(ident,{'action':'relocate','pose':[5,0,0]})
    atomic_json(root/'result.json',{'status':'succeeded'})
    assert not api.session(ident)['safe_to_adjust']
    with pytest.raises(ValueError):api.control(ident,{'action':'pause'})


def test_unknown_route_and_arbitrary_parameters(tmp_path,profile):
    app=WorkcellWSGI(Service(tmp_path,profile))
    code,_=wsgi(app,'/api/workcell/iterate/randomize',{'seed':42,'geometry':{'workspace':[0,999,0,999]}})
    assert code.startswith('400')
    code,_=wsgi(app,'/api/workcell/iterate/missing',{})
    assert code.startswith('404')
