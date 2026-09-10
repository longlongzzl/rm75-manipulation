import copy
import importlib.util
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import pytest
from rm75_app.workcell.io import atomic_json,read_json
from rm75_app.workcell.service import WorkcellService
from rm75_app.workcell.worker import run_pusht
from rm75_app.workcell.events import EventLog,StopToken
from rm75_app.workcell.spec import validate_spec
from rm75_app.magnetic.generation import compile_selection

ROOT=Path(__file__).resolve().parents[2]


def test_real_service_worker_sequence_preview(tmp_path,profile):
    path=tmp_path/'profile.json';atomic_json(path,profile)
    service=WorkcellService(ROOT,path,allow_real=False)
    try:
        spec=dict(task='pickplace',mode='preview',parameters=dict(object_names=['tennis','bi'],automatic_order=True))
        ident=service.submit(spec)['job_id']
        service._thread.join(timeout=10);assert not service.active
        job=service.job(ident)
        assert job['status']=='command_completed_unverified'
        assert job['result']['source_order']==['bi','tennis'] and job['result']['task_success'] is None
    finally:service.close()


def test_real_service_worker_generated_preview(tmp_path,profile,library,proposal):
    path=tmp_path/'profile.json';atomic_json(path,profile)
    service=WorkcellService(ROOT,path,allow_real=False)
    data=compile_selection(library,proposal,board_id='grid_3x3')
    try:
        ident=service.submit(dict(task='magnetic',mode='preview',parameters={'design':data['design'],'generation_proof':data['proof']}))['job_id']
        service._thread.join(timeout=10);assert not service.active
        job=service.job(ident)
        assert job['status']=='command_completed_unverified'
        assert job['result']['task_success'] is None
    finally:service.close()


def test_physics_dispatch_preserves_current_model_and_policy_file(tmp_path,profile,monkeypatch):
    captured=[]
    fake=SimpleNamespace(run=lambda spec,p,cfg,stop,events:captured.append((spec,p,cfg,events)) or {'command_success':True,'task_success':None})
    monkeypatch.setitem(sys.modules,'rm75_app.pusht.physics',fake)
    spec=validate_spec(dict(task='pusht',mode='sim',parameters=dict(goal_pose=[.4,0,0],geometry_id='wide',
        simulation_backend='full_arm_physics',maximum_push_length_m=.06,run_until_goal=True)),profile)
    events=EventLog(tmp_path)
    run_pusht(spec,profile,StopToken(),events)
    assert len(captured)==1 and captured[0][2].bar_width_m==.12 and captured[0][2].maximum_push_length_m==.06
    assert read_json(tmp_path/'session_policy.json')['run_until_goal'] is True
    assert captured[0][2].response_fits==()


def test_native_dispatch_preserves_original_main_and_per_job_profile(tmp_path,profile,monkeypatch):
    from rm75_app.workcell.iteration_workflows import run
    captured=[]
    def native(spec,p,*args):
        captured.append((copy.deepcopy(spec),copy.deepcopy(p)))
        return {'command_success':False,'native_full_world_validation':{'source_outcomes':[
            dict(source='bi',success=True,foreground=True),dict(source='tennis',success=False,foreground=True)]}}
    monkeypatch.setitem(sys.modules,'rm75_app.workcell.legacy',SimpleNamespace(run_working=native))
    result=run(dict(task='pickplace',mode='sim',parameters=dict(object_names=['tennis','bi'],automatic_order=True)),
               profile,ROOT,tmp_path,StopToken(),EventLog(tmp_path))
    assert captured[0][0]['parameters']=={'object_name':'bi'}
    assert captured[0][1]['pickplace']['frozen_source_order']==['bi','tennis']
    assert result['sequence_summary']['pending']==['tennis'] and result['command_success'] is False


def test_random_suite_generate_only_no_devices(tmp_path,profile):
    path=tmp_path/'profile.json';atomic_json(path,profile);out=tmp_path/'suite'
    result=subprocess.run([sys.executable,'tools/run_pusht_random_suite.py','--profile',str(path),'--output',str(out),
                           '--seed','77','--count','8','--geometry-id','wide'],cwd=ROOT,capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr
    assert len(read_json(out/'cases.json')['cases'])==8
    assert not (out/'results.json').exists()


def test_real_service_worker_interactive_surrogate_not_physics(tmp_path,profile):
    path=tmp_path/'profile.json';atomic_json(path,profile)
    service=WorkcellService(ROOT,path,allow_real=False)
    try:
        spec=dict(task='pusht',mode='sim',parameters=dict(initial_pose=[.35,0,0],goal_pose=[.36,0,0],
                  simulation_backend='surrogate',run_until_goal=True,maximum_push_length_m=.05))
        ident=service.submit(spec)['job_id'];service._thread.join(timeout=15)
        assert not service.active
        result=service.job(ident)['result']
        assert result['status']=='succeeded' and result['verification']=='surrogate_pose'
        assert not result['hardware_connected'] and not result['hardware_profile_qualified']
        assert result['steps']>=1
    finally:service.close()
