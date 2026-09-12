"""Regressions for the real-machine console acceptance findings.

CPU/WSGI/component substitutes only. No GPU, native engine or model request.
"""
import copy
import hashlib
import io
import os
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.profile_discovery import discover
from rm75_app.workcell.task_diagnostics import scoped_profile, PICKPLACE_ONLY
from rm75_app.workcell.server import WorkcellWSGI


def make_templates(fixture, monkeypatch, *, bad=False):
    source = Path(fixture.service.profile['magnetic'].pop('fixed_scene'))
    lib = {'boards': {'arc': {'templates': [{'native_recipe': {
        'fixed_scene': str(source) if not bad else str(source.parent/'missing.json'),
        'fixed_scene_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}}]}}}
    monkeypatch.setattr(sys.modules['rm75_app.magnetic.generation'], 'load_library', lambda _: lib)


def test_template_only_bootstrap_is_not_false_red(fixture, monkeypatch):
    make_templates(fixture, monkeypatch)
    row = next(x for x in fixture.bootstrap()['checks'] if x['key']=='magnetic.scene')
    assert row['status']=='ready' and '提交时' in row['hint']
    assert not fixture.service.submissions


def test_manual_design_still_requires_its_own_scene(fixture, monkeypatch):
    make_templates(fixture, monkeypatch)
    spec={'task':'magnetic','mode':'sim','parameters':{'design':{}}}
    row=next(x for x in fixture.configuration_checks('magnetic','sim',spec) if x['key']=='magnetic.scene')
    assert row['status']=='error'


def test_missing_template_scene_stays_red(fixture, monkeypatch):
    make_templates(fixture, monkeypatch, bad=True)
    assert next(x for x in fixture.bootstrap()['checks'] if x['key']=='magnetic.scene')['status']=='error'


def test_invalid_library_cannot_grant_readiness(fixture, monkeypatch):
    fixture.service.profile['magnetic'].pop('fixed_scene')
    def invalid(_): raise ValueError('changed original base')
    monkeypatch.setattr(sys.modules['rm75_app.magnetic.generation'], 'load_library', invalid)
    assert next(x for x in fixture.bootstrap()['checks'] if x['key']=='magnetic.scene')['status']=='error'


def test_geometry_dedup_does_not_mutate_source_config(fixture, monkeypatch):
    source=fixture.iteration.features();source['geometry_ids']=['original','original','wide','wide']
    before=copy.deepcopy(source)
    monkeypatch.setattr(fixture.iteration,'features',lambda:source)
    assert fixture.bootstrap()['features']['geometry_ids']==['original','wide']
    assert source==before


def call(app,path,method='GET'):
    headers=[]
    env={'PATH_INFO':path,'REQUEST_METHOD':method,'HTTP_HOST':'127.0.0.1:7863',
         'wsgi.input':io.BytesIO(b''),'CONTENT_LENGTH':'0'}
    data=b''.join(app(env,lambda code,h:headers.append((code,dict(h)))))
    return headers[0], data


def test_favicon_and_svg_are_explicit_safe_routes(fixture):
    app=WorkcellWSGI(fixture.service)
    (code,head),data=call(app,'/favicon.ico')
    assert code.startswith('204') and not data
    (code,head),data=call(app,'/workcell/console/icon.svg')
    assert code.startswith('200') and head['Content-Type'].startswith('image/svg+xml')
    assert b'<script' not in data and b'<svg' in data
    assert call(app,'/workcell/console/icon.svg','POST')[0][0].startswith('405')
    assert b'rel="icon"' in call(app,'/workcell/console/')[1]


def test_favicon_does_not_override_mounted_legacy_server(fixture):
    seen=[]
    def legacy(env,start):seen.append(env['PATH_INFO']);start('200 OK',[]);return [b'legacy icon']
    app=WorkcellWSGI(fixture.service,legacy)
    assert call(app,'/favicon.ico')[1]==b'legacy icon' and seen==['/favicon.ico']


def test_profile_discovery_arbitrary_names_and_history_pruning(tmp_path):
    data={'schema':'rm75_workcell_machine_v1','magnetic':{'design_library':'local-only'},
          'pusht':{'physics':{'enabled_backends':['full_arm_physics']}}}
    actual=tmp_path/'runtime_data/console_launch_20260911/console_profile_r2_nodiag.json'
    atomic_json(actual,data)
    atomic_json(tmp_path/'runtime_data/workcell/jobs/a/machine_profile.json',data)
    atomic_json(tmp_path/'runtime_data/unrelated.json',{'secret':'do not print'})
    result=discover(tmp_path)
    assert [r['path'] for r in result['profiles']]==[str(actual)]
    assert result['profiles'][0]['has_jimu_library'] and not result['selected_automatically']
    assert 'secret' not in str(result)


def test_profile_discovery_explicit_root_and_hard_budget(tmp_path):
    data={'schema':'rm75_workcell_machine_v1'}
    outside=tmp_path/'external/anything.json';atomic_json(outside,data)
    for i in range(8):atomic_json(tmp_path/f'runtime_data/x{i}.json',data)
    result=discover(tmp_path,explicit=[outside],max_entries=1)
    assert result['scan_truncated'] and str(outside) in [r['path'] for r in result['profiles']]
    assert discover(tmp_path,search_roots=[outside])['profiles'][0]['path']==str(outside)


def test_profile_discovery_skips_symlinks_and_exposes_depth_limit(tmp_path):
    actual=tmp_path/'original.json';atomic_json(actual,{'schema':'rm75_workcell_machine_v1'})
    folder=tmp_path/'runtime_data';folder.mkdir()
    (folder/'symlink.json').symlink_to(actual)
    (folder/'nested').mkdir()
    result=discover(tmp_path,max_depth=0)
    assert result['profiles']==[] and result['scan_truncated']


@pytest.mark.parametrize('flag',PICKPLACE_ONLY)
def test_shared_profile_diagnostics_are_task_scoped(flag):
    raw={'pickplace':{flag:True,'native_args':['--keep']},
         'magnetic':{'simulation_contact_policy':'strict'},'hardware':{'hardware_reviewed':False}}
    before=copy.deepcopy(raw)
    adjusted,ignored=scoped_profile({'task':'magnetic'},raw)
    assert flag not in adjusted['pickplace'] and ignored==[flag]
    assert raw==before and adjusted['magnetic']==raw['magnetic'] and adjusted['hardware']==raw['hardware']
    adjusted,ignored=scoped_profile({'task':'pickplace'},raw)
    assert adjusted==raw and not ignored


def test_native_worker_facade_receives_scoped_copy(fixture,monkeypatch,tmp_path):
    from rm75_app.workcell.iteration_workflows import run
    calls=[]
    def native(spec,profile,*args):
        calls.append(profile)
        assert not profile['pickplace'].get('audit_current_table_failures')
        return {'command_success':True,'task_success':None}
    fake=ModuleType('rm75_app.workcell.legacy');fake.run_working=native
    monkeypatch.setitem(sys.modules,fake.__name__,fake)
    profile=copy.deepcopy(fixture.service.profile);profile['pickplace']['audit_current_table_failures']=True
    log=[]
    events=SimpleNamespace(emit=lambda *a,**kw:log.append((a,kw)))
    result=run({'task':'magnetic','mode':'sim','parameters':{'design':{}}},profile,
               tmp_path,tmp_path/'job',None,events)
    assert calls and profile['pickplace']['audit_current_table_failures'] is True
    assert log[0][0]==('unrelated_task_diagnostic_ignored',) and result['task_success'] is None


def test_browser_dependency_failure_leaves_output_absent(tmp_path,monkeypatch):
    from tools.check_workcell_console import main
    monkeypatch.setitem(sys.modules,'playwright.sync_api',None)
    output=tmp_path/'evidence'
    with pytest.raises(RuntimeError,match='Playwright is unavailable'):
        main(['--output',str(output)])
    assert not output.exists()


def test_browser_launch_failure_leaves_output_reusable(tmp_path,monkeypatch):
    import tools.check_workcell_console as check
    class PW:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        chromium=SimpleNamespace(launch=lambda **kw:(_ for _ in ()).throw(RuntimeError('launch failed')))
    monkeypatch.setattr(check,'playwright_api',lambda:(lambda:PW(),object()))
    monkeypatch.setattr(check,'chromium_options',lambda *a:{'headless':True})
    output=tmp_path/'evidence'
    for _ in range(2):
        with pytest.raises(RuntimeError,match='launch failed'):check.main(['--output',str(output)])
        assert not output.exists()


def test_browser_explicit_executable_checked_before_evidence(tmp_path):
    from tools.console_browser_support import chromium_options
    with pytest.raises(RuntimeError,match='No executable Chromium'):
        chromium_options(None,tmp_path/'missing')
    browser=tmp_path/'browser';browser.write_text('#!/bin/sh\nexit 0\n');browser.chmod(0o755)
    assert chromium_options(None,browser)['executable_path']==str(browser)


def test_anthropic_explicit_transport_blocks_sdk_environment_mount_bug(monkeypatch):
    from rm75_app.magnetic.completion_provider import AnthropicJsonClient
    monkeypatch.setenv('ANTHROPIC_AUTH_TOKEN','test-not-a-secret')
    monkeypatch.setenv('ANTHROPIC_BASE_URL','https://example.invalid')
    monkeypatch.setenv('ANTHROPIC_MODEL','configured')
    monkeypatch.setenv('ALL_PROXY','socks://127.0.0.1:7897/')
    monkeypatch.setenv('HTTPS_PROXY','socks://not-used')
    before=dict(os.environ);calls=[]
    # Emulate the verified v1.4.0 constructor's key-presence branch. This is NOT
    # an installed Anthropic SDK test or a model network call.
    def default(**kwargs):
        if 'transport' not in kwargs:raise ValueError('environment proxy map parsed despite trust_env=False')
        calls.append(kwargs);return SimpleNamespace(close=lambda:None)
    module=ModuleType('anthropic');module.DefaultHttpxClient=default
    module.Anthropic=lambda **kwargs:SimpleNamespace(close=lambda:kwargs['http_client'].close())
    monkeypatch.setitem(sys.modules,'anthropic',module)
    with pytest.raises(ValueError):default(trust_env=False)
    client=AnthropicJsonClient({'proxy':'http://127.0.0.1:7897'})._make_client();client.close()
    assert calls[0]['transport'] is None and calls[0]['trust_env'] is False
    assert calls[0]['follow_redirects'] is False and calls[0]['proxy']=='http://127.0.0.1:7897'
    assert dict(os.environ)==before


def test_sdk_init_errors_are_separate_and_redacted(monkeypatch):
    from rm75_app.magnetic.completion_provider import AnthropicJsonClient,CompletionClientInitError
    monkeypatch.setenv('TEST_LLM_TOKEN','unit-test-not-a-secret')
    def fail():raise ValueError('secret-URL-and-key')
    c=AnthropicJsonClient({'model':'test','api_base':'https://example.invalid','auth_token_env':'TEST_LLM_TOKEN'},client_factory=fail)
    with pytest.raises(CompletionClientInitError) as error:c([{'role':'user','content':'JSON'}])
    assert error.value.code=='LLM_CLIENT_INIT' and 'secret-URL' not in str(error.value)


def test_client_init_failure_is_actionable_without_raw_sdk_text(fixture):
    ident='f'*32
    root=fixture.service.root/'design_generations'/ident;root.mkdir(parents=True)
    fixture.iteration._jobs[ident]={'status':'failed','error_type':'CompletionClientInitError','error':'generic'}
    result=fixture.generation_status(ident)
    assert result['error_code']=='LLM_CLIENT_INIT' and 'magnetic.llm.proxy' in result['error']
    assert fixture.iteration._jobs[ident]['error']=='generic'


def test_browser_output_race_does_not_write_another_runs_report(tmp_path,monkeypatch):
    import tools.check_workcell_console as check
    output=tmp_path/'evidence'
    def launch(**kwargs):
        output.mkdir();(output/'report.json').write_text('other run')
        return SimpleNamespace()
    class PW:
        chromium=SimpleNamespace(launch=launch)
        def __enter__(self):return self
        def __exit__(self,*args):pass
    monkeypatch.setattr(check,'playwright_api',lambda:(lambda:PW(),object()))
    monkeypatch.setattr(check,'chromium_options',lambda *a:{})
    with pytest.raises(FileExistsError):check.main(['--output',str(output)])
    assert (output/'report.json').read_text()=='other run'
