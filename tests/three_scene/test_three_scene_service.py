import copy,io,time
from pathlib import Path
import pytest
from rm75_app.workcell.io import atomic_json,loads,dumps
from rm75_app.workcell.service import WorkcellService
from rm75_app.workcell.server import WorkcellWSGI
@pytest.fixture
def service(tmp_path,profile):
    app=tmp_path/'app';app.mkdir();(app/'rm75_app').symlink_to(Path(__file__).resolve().parents[2]/'rm75_app',target_is_directory=True);path=tmp_path/'profile.json';atomic_json(path,profile);svc=WorkcellService(app,path);yield svc;svc.close()
def wait(service,job,timeout=10):
    deadline=time.monotonic()+timeout
    while service.active and time.monotonic()<deadline:time.sleep(.03)
    assert not service.active;return service.job(job)
@pytest.mark.parametrize('task',['pickplace','magnetic','pusht'])
def test_actual_subprocess_preview_all_three(service,design,task):
    params={'pickplace':{'object_name':'carriot'},'magnetic':{'design':design},'pusht':{'goal_pose':[.38,0,0]}}[task];job=service.submit({'task':task,'mode':'preview','parameters':params})['job_id'];result=wait(service,job);assert result['result']['command_success'] is True and result['result']['task_success'] is None
def test_actual_pusht_subprocess_full_loop(service):
    job=service.submit({'task':'pusht','mode':'sim','parameters':{'initial_pose':[.35,0,0],'goal_pose':[.38,.015,.1]}})['job_id'];result=wait(service,job);assert result['result']['task_success'] is True and any(e['kind']=='observation' for e in result['events'])
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
