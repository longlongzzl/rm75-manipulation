import copy,io,time
from pathlib import Path
import pytest
from rm75_app.workcell.io import atomic_json,loads,dumps
from rm75_app.workcell.service import WorkcellService
from rm75_app.workcell.server import WorkcellWSGI
@pytest.fixture
def service(tmp_path,profile):
    app=tmp_path/'app';app.mkdir();package=app/'rm75_app';package.mkdir()
    # This fixture deliberately has no installed native source. Linking the
    # whole package accidentally imports the developer's real vendored snapshot.
    for source in (Path(__file__).resolve().parents[2]/'rm75_app').iterdir():
        if source.name not in ('_vendor','__pycache__'):
            (package/source.name).symlink_to(source,target_is_directory=source.is_dir())
    path=tmp_path/'profile.json';atomic_json(path,profile);svc=WorkcellService(app,path);yield svc;svc.close()
def wait(service,job,timeout=10):
    deadline=time.monotonic()+timeout
    while service.active and time.monotonic()<deadline:time.sleep(.03)
    assert not service.active;return service.job(job)
@pytest.mark.parametrize('task',['pickplace','magnetic','pusht'])
def test_actual_subprocess_preview_all_three(service,design,task):
    params={'pickplace':{'object_name':'carriot'},'magnetic':{'design':design},'pusht':{'goal_pose':[.38,0,0]}}[task];job=service.submit({'task':task,'mode':'preview','parameters':params})['job_id'];result=wait(service,job);assert result['result']['command_success'] is True and result['result']['task_success'] is None
def test_actual_pusht_subprocess_full_loop(service):
    job=service.submit({'task':'pusht','mode':'sim','parameters':{'initial_pose':[.35,0,0],'goal_pose':[.38,.015,.1]}})['job_id'];result=wait(service,job);assert result['result']['task_success'] is True and any(e['kind']=='observation' for e in result['events'])


@pytest.mark.parametrize('backend',['tool_only_physics','full_arm_physics'])
def test_physics_wsgi_submit_reaches_actual_preview_worker_without_hardware(service,backend):
    service.profile['pusht']['physics']={'enabled_backends':[backend]}
    atomic_json(service.profile_path,service.profile)
    spec=dict(task='pusht',mode='preview',parameters=dict(goal_pose=[.38,0,0],simulation_backend=backend))
    code,body=call(WorkcellWSGI(service),'/api/workcell/jobs','POST',{'spec':spec},service.csrf)
    assert code=='200 OK'
    result=wait(service,loads(body.decode())['job_id'])['result']
    assert result['command_success'] and result['simulation_backend']==backend
    assert not result['hardware_connected'] and not result['hardware_profile_qualified']
def test_original_missing_source_fails_explicitly_not_fake_success(service):
    job=service.submit({'task':'pickplace','mode':'sim','parameters':{'object_name':'carriot'}})['job_id'];result=wait(service,job);assert result['status']=='failed' and result['result']['task_success'] is None
def test_real_is_disabled_without_explicit_server_permission(service):
    spec={'task':'pickplace','mode':'real','parameters':{'object_name':'bi'}}
    with pytest.raises(PermissionError):service.arm(spec,'我确认现场安全并允许本次真机运行')
def call(app,path,method='GET',payload=None,token='',origin='http://127.0.0.1:7861'):
    raw=b'' if payload is None else dumps(payload).encode();env={'PATH_INFO':path,'REQUEST_METHOD':method,'HTTP_HOST':'127.0.0.1:7861','HTTP_ORIGIN':origin,'HTTP_X_WORKCELL_TOKEN':token,'CONTENT_LENGTH':str(len(raw)),'wsgi.input':io.BytesIO(raw)};status=[];result=b''.join(app(env,lambda code,headers:status.append(code)));return status[0],result
def test_wsgi_static_and_csrf_origin(service):
    app=WorkcellWSGI(service);code,body=call(app,'/workcell/');assert code=='200 OK' and '三场景'.encode() in body;assert loads(call(app,'/api/workcell/info')[1].decode())['tasks']==['pickplace','magnetic','pusht']
    spec={'spec':{'task':'pickplace','mode':'preview','parameters':{'object_name':'bi'}}};assert call(app,'/api/workcell/jobs','POST',spec)[0].startswith('403');code,body=call(app,'/api/workcell/jobs','POST',spec,service.csrf);assert code=='200 OK';wait(service,loads(body.decode())['job_id'])


@pytest.mark.parametrize('unresponsive',[False,True])
def test_physics_stop_is_cooperative_but_unresponsive_group_is_bounded(service,monkeypatch,unresponsive):
    import subprocess,signal
    from types import SimpleNamespace as NS
    calls=[];directory=service.root/'jobs'/'physics';directory.mkdir(parents=True)
    atomic_json(directory/'request.json',dict(task='pusht',mode='sim',parameters={'simulation_backend':'full_arm_physics'}))
    def waited(timeout):
        calls.append(('wait',timeout))
        if unresponsive:raise subprocess.TimeoutExpired('fake physics process',timeout)
    service.active='physics';service.process=NS(pid=123,poll=lambda:None,wait=waited)
    monkeypatch.setattr('rm75_app.workcell.service.os.killpg',lambda pid,sig:calls.append(('signal',sig)))
    class Thread:
        def __init__(self,target,daemon):self.target=target
        def start(self):self.target()
    monkeypatch.setattr('rm75_app.workcell.service.threading.Thread',Thread)
    try:
        result=service.cancel('physics')
        assert (directory/'STOP').is_file() and result['physics_cooperative_stop']
        assert calls==[('wait',5)]+([('signal',signal.SIGKILL)] if unresponsive else [])
    finally:service.active=None;service.process=None
