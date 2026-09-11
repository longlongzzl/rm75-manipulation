import io
import json
import threading
import time
from pathlib import Path
import pytest
from rm75_app.workcell.console_api import ConsoleAPI,redacted
from rm75_app.workcell.io import atomic_json,read_json
from rm75_app.workcell.server import WorkcellWSGI
from .fixtures import design


def request(name='bi',mode='preview'):
    return {'task':'pickplace','mode':mode,'parameters':{'object_name':name}}


def test_bootstrap_public_metadata_no_hardware(fixture):
    data=fixture.bootstrap();assert data['mode_policy']['allowed']==['preview','sim']
    assert not data['mode_policy']['real_submission_enabled'] and data['generation']['browser_deadline'] is None
    assert data['generation']['estimated_budget_s']==630
    assert 'hardware' not in data and 'profile' not in data and not fixture.service.submissions


def test_partial_feature_failure_does_not_kill_console(fixture,monkeypatch):
    monkeypatch.setattr(fixture.iteration,'features',lambda:(_ for _ in ()).throw(ValueError('bad T config')))
    b=fixture.bootstrap();assert b['features']['errors'] and b['info']['pickplace_objects']


@pytest.mark.parametrize('mode',['real','unknown',None])
def test_real_and_unknown_modes_never_dispatch(fixture,mode):
    with pytest.raises(PermissionError):fixture.submit({'request_id':'a'*32,'spec':request(mode=mode)})
    assert not fixture.service.submissions


def test_preflight_has_no_side_effects(fixture):
    result=fixture.preflight({'spec':request()})
    assert result['valid'] and result['hardware_contacted'] is False
    assert not fixture.service.submissions and not fixture.service.active


def test_missing_native_environment_blocks_sim_not_preview(fixture):
    fixture.service.profile['pickplace']['python']='/missing/python'
    sim=fixture.preflight({'spec':request(mode='sim')});pre=fixture.preflight({'spec':request()})
    assert not sim['valid'] and pre['valid']


def test_repeated_post_returns_same_job_even_when_busy(fixture):
    fixture.service.delay=2
    p={'request_id':'a'*32,'spec':request()}
    first=fixture.submit(p);again=fixture.submit(p)
    assert first==again and len(fixture.service.submissions)==1
    assert fixture.get('submissions/'+'a'*32)['job_id']==first['job_id']


def test_duplicate_identity_rejects_changed_spec(fixture):
    fixture.submit({'request_id':'b'*32,'spec':request()})
    with pytest.raises(ValueError):fixture.submit({'request_id':'b'*32,'spec':request('shuazi')})
    assert len(fixture.service.submissions)==1


def test_concurrent_duplicate_posts_dispatch_once(fixture):
    p={'request_id':'c'*32,'spec':request()};rows=[];errors=[]
    def work():
        try:rows.append(fixture.submit(p))
        except Exception as e:errors.append(e)
    threads=[threading.Thread(target=work) for _ in range(8)]
    for t in threads:t.start()
    for t in threads:t.join()
    assert not errors and len(rows)==8 and len({r['job_id'] for r in rows})==1
    assert len(fixture.service.submissions)==1


def test_uncertain_submission_never_retried(fixture,monkeypatch):
    calls=[]
    def fail(*a):calls.append(1);raise RuntimeError('uncertain process start')
    monkeypatch.setattr(fixture.service,'submit',fail)
    p={'request_id':'d'*32,'spec':request()}
    with pytest.raises(RuntimeError):fixture.submit(p)
    assert fixture.submit(p)['status']=='uncertain' and len(calls)==1


def test_restarted_facade_returns_journal_ack(fixture):
    p={'request_id':'e'*32,'spec':request()};before=fixture.submit(p)
    again=ConsoleAPI(fixture.service,fixture.iteration).submit(p)
    assert again==before and len(fixture.service.submissions)==1


@pytest.mark.parametrize('ident',['../escape','A'*32,'a'*31,'a'*33,'',None,'x'*32])
def test_invalid_id_no_write(fixture,ident):
    with pytest.raises(ValueError):fixture.submit({'request_id':ident,'spec':request()})
    assert not fixture.service.submissions


def test_symlink_journal_refused(fixture,tmp_path):
    group=fixture.service.root/'console_submissions';group.mkdir()
    outside=tmp_path/'secret.json';atomic_json(outside,{'sensitive':'data'})
    (group/('f'*32+'.json')).symlink_to(outside)
    with pytest.raises(ValueError):fixture.get('submissions/'+'f'*32)


def test_unknown_unmanaged_job_is_not_reported_stopped(fixture):
    ident='a'*32;root=fixture.service.root/'jobs'/ident
    atomic_json(root/'request.json',request())
    assert fixture.job(ident)['status']=='untracked'
    assert not fixture.job(ident)['managed_active']


def test_history_bounded_and_malformed_entries_ignored(fixture):
    for i in range(30):
        path=fixture.service.root/'jobs'/f'{i:032x}'
        atomic_json(path/'request.json',request())
        atomic_json(path/'result.json',{'status':'command_completed_unverified'})
    path=fixture.service.root/'jobs'/('z'*32);path.mkdir();(path/'request.json').write_text('garbage')
    assert len(fixture.history()['jobs'])==24


def test_generation_long_wait_and_duplicate_identity(fixture):
    fixture.iteration.delay=2
    p={'request_id':'1'*32,'request':{'board_id':'arc','prompt':'test','piece_budget':3}}
    a=fixture.generation_start(p);b=fixture.generation_start(p)
    assert a==b and len(fixture.iteration.calls)==1
    ident=a['generation_id'];root=fixture.service.root/'design_generations'/ident
    meta=read_json(root/'console_meta.json');meta['created_at']-=500;atomic_json(root/'console_meta.json',meta)
    result=fixture.generation_status(ident)
    assert result['status']=='running' and time.time()-result['created_at']>=500
    assert result['auto_execute'] is False


def test_abandoned_generation_not_adopted_even_after_model_completion(fixture):
    fixture.iteration.delay=2
    p={'request_id':'2'*32,'request':{'board_id':'arc','prompt':'test','piece_budget':3}}
    ident=fixture.generation_start(p)['generation_id']
    assert fixture.abandon_generation(ident)['status']=='abandoned'
    fixture.iteration.finish(ident,p['request'])
    assert fixture.generation_status(ident)['status']=='abandoned'
    assert not fixture.service.submissions


def test_restart_with_missing_worker_is_interrupted_not_new_call(fixture):
    fixture.iteration.delay=2
    p={'request_id':'3'*32,'request':{'board_id':'arc','prompt':'test','piece_budget':3}}
    ident=fixture.generation_start(p)['generation_id'];fixture.iteration._jobs.clear()
    assert fixture.generation_status(ident)['status']=='interrupted'
    assert len(fixture.iteration.calls)==1


def test_unknown_generation_not_synthesized(fixture):
    with pytest.raises(FileNotFoundError):fixture.generation_status('4'*32)


def test_templates_carry_matching_recipe_proof_without_llm(fixture):
    result=fixture.templates('arc')['templates'][0]
    assert result['proof']['board_id']=='arc' and result['origin']=='original_template_no_llm'
    assert not fixture.iteration.calls


def test_export_redacts_credentials_but_no_full_profile(fixture,monkeypatch):
    monkeypatch.setenv('TEST_API_KEY','super-private-token')
    assert redacted({'error':'got super-private-token','api_key':'something'})=={'error':'got [redacted]','api_key':'[redacted]'}
    ident=fixture.submit({'request_id':'5'*32,'spec':request()})['job_id']
    report=fixture.report(ident)
    assert 'profile' not in report and 'log' not in report and report['request']==request()


def wsgi(app,path,body=None,token='fixture-token',host='127.0.0.1:7861',origin=None):
    payload=json.dumps(body).encode() if body is not None else b'';head=[]
    env={'PATH_INFO':path,'REQUEST_METHOD':'GET' if body is None else 'POST','HTTP_HOST':host,
         'CONTENT_LENGTH':str(len(payload)),'wsgi.input':io.BytesIO(payload),'HTTP_X_WORKCELL_TOKEN':token}
    if origin is not None:env['HTTP_ORIGIN']=origin
    raw=b''.join(app(env,lambda status,headers:head.append((status,headers))))
    return int(head[0][0][:3]),raw,dict(head[0][1])


def test_real_wsgi_new_console_assets_and_csp(fixture):
    app=WorkcellWSGI(fixture.service)
    for asset in ('','index.html','app.js','state.js','views.js','console.css'):
        code,data,headers=wsgi(app,'/workcell/console/'+asset)
        assert code==200 and data and 'unsafe-inline' not in headers['Content-Security-Policy']
    code,body,_=wsgi(app,'/api/workcell/console/bootstrap')
    assert code==200 and json.loads(body)['mode_policy']['real_submission_enabled'] is False


@pytest.mark.parametrize('token,host,origin',[('wrong','127.0.0.1:7861',None),('fixture-token','attacker.example',None),('fixture-token','127.0.0.1:7861','https://attacker.example')])
def test_new_posts_inherit_same_origin_token_guard(fixture,token,host,origin):
    code,_,_=wsgi(WorkcellWSGI(fixture.service),'/api/workcell/console/jobs',{'request_id':'a'*32,'spec':request()},token,host,origin)
    assert code==403 and not fixture.service.submissions


def test_console_get_dns_rebinding_rejected(fixture):
    code,_,_=wsgi(WorkcellWSGI(fixture.service),'/api/workcell/console/bootstrap',host='evil.example')
    assert code==403


def test_raw_real_request_cannot_use_new_endpoint(fixture):
    code,_,_=wsgi(WorkcellWSGI(fixture.service),'/api/workcell/console/jobs',{'request_id':'6'*32,'spec':request(mode='real')})
    assert code==403 and not fixture.service.submissions


def test_existing_stop_endpoint_still_calls_same_service(fixture):
    ident=fixture.submit({'request_id':'7'*32,'spec':request(mode='sim')})['job_id']
    app=WorkcellWSGI(fixture.service)
    code,_,_=wsgi(app,f'/api/workcell/jobs/{ident}/stop',{})
    assert code==200 and fixture.service.stops==[ident]


def test_no_arbitrary_static_paths(fixture):
    code,_,_=wsgi(WorkcellWSGI(fixture.service),'/workcell/console/../../machine_profile.json')
    assert code==404


def test_numeric_roundtrip_restores_exact_compiler_design_before_dispatch(fixture):
    from .fixtures import compile_selection,library
    from rm75_app.workcell.io import digest
    proposal={'board_id':'arc','template_id':'arc_00','title':'fixture','selected_roles':['wall_0','wall_1','wall_2'],'explanation':'test'}
    original=compile_selection(library(),proposal,board_id='arc')
    # JavaScript JSON.stringify serializes integral floating point values as ints.
    def js_roundtrip(v):
        if isinstance(v,dict):return {k:js_roundtrip(x) for k,x in v.items()}
        if isinstance(v,list):return [js_roundtrip(x) for x in v]
        if type(v) is float and v.is_integer():return int(v)
        return v
    rounded=js_roundtrip(original['design'])
    assert digest(rounded)!=digest(original['design'])
    requested={'task':'magnetic','mode':'preview','parameters':{'design':rounded,'generation_proof':original['proof']}}
    checked=fixture.preflight({'spec':requested})
    assert digest(checked['spec']['parameters']['design'])==original['proof']['design_digest']
    fixture.submit({'request_id':'8'*32,'spec':requested})
    assert digest(fixture.service.submissions[-1]['parameters']['design'])==original['proof']['design_digest']


@pytest.mark.parametrize('change',['coordinate','locked_number','missing_key','new_key','order'])
def test_roundtrip_fix_does_not_accept_geometry_or_schema_changes(fixture,change):
    result=fixture.templates('arc')['templates'][0]
    d=result['design']
    if change=='coordinate':d['pieces'][0]['center'][0]+=1e-9
    if change=='locked_number':d['pieces'][0]['locked']=1
    if change=='missing_key':d['pieces'][0].pop('id')
    if change=='new_key':d['untrusted']=1
    if change=='order':d['pieces'].reverse()
    with pytest.raises(ValueError):
        fixture.preflight({'spec':{'task':'magnetic','mode':'preview','parameters':{'design':d,'generation_proof':result['proof']}}})


def test_busy_cross_process_lease_blocks_submit_without_deleting_lock(fixture):
    import fcntl
    lock=fixture.service.root/'robot.lock'
    with lock.open('w') as f:
        f.write('owner');f.flush();fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result=fixture.preflight({'spec':request()})
        assert result['busy']
        with pytest.raises(RuntimeError):fixture.submit({'request_id':'9'*32,'spec':request()})
        assert lock.read_text()=='owner'
    assert fixture.preflight({'spec':request()})['busy'] is False
