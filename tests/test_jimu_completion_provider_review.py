"""Provider wire tests using an SDK substitute, never a commercial API call."""
import ast
import copy
import json
import os
from pathlib import Path
from types import SimpleNamespace, ModuleType
import sys
import pytest
from rm75_app.magnetic.completion_provider import AnthropicJsonClient, completion_client, DisabledCompletion


@pytest.fixture
def env(monkeypatch):
    for name in list(os.environ):
        if name.startswith(('RM75_LLM_', 'RM75_VLM_', 'ANTHROPIC_')) or name=='OPENAI_API_KEY':
            monkeypatch.delenv(name)
    monkeypatch.setenv('ANTHROPIC_AUTH_TOKEN','local-test-not-a-real-token')
    monkeypatch.setenv('ANTHROPIC_BASE_URL','https://example.invalid/anthropic')
    monkeypatch.setenv('ANTHROPIC_DEFAULT_OPUS_MODEL','machine-configured-model')
    monkeypatch.setattr('importlib.util.find_spec',lambda name:object() if name=='anthropic' else None)
    return monkeypatch


def sdk_response(stop='end_turn',blocks=None):
    return SimpleNamespace(stop_reason=stop,content=blocks or [SimpleNamespace(type='text',text='{"selected_roles":[]}')])


class FakeClient:
    def __init__(self,response):self.response=response;self.requests=[];self.closed=False
    @property
    def messages(self):return self
    def stream(self,**kwargs):
        self.requests.append(kwargs);owner=self
        class Stream:
            def __enter__(self):return self
            def __exit__(self,*args):return False
            def __iter__(self):return iter([object()])
            def get_final_message(self):return owner.response
        return Stream()
    def close(self):self.closed=True


def test_web_factory_uses_existing_anthropic_environment_without_openai_key(env):
    c=completion_client({})
    assert isinstance(c,AnthropicJsonClient)
    assert c.readiness()['configured'] and c.model=='machine-configured-model'
    assert c.base=='https://example.invalid/anthropic'
    assert c.key_env=='ANTHROPIC_AUTH_TOKEN'
    assert 'local-test' not in json.dumps(c.readiness())


def test_real_sdk_constructor_does_not_mutate_global_proxies(env):
    env.setenv('ALL_PROXY','socks://not-used');env.setenv('HTTPS_PROXY','socks://not-used')
    env.setenv('RM75_LLM_PROXY','http://127.0.0.1:7897');before=dict(os.environ);calls=[]
    module=ModuleType('anthropic')
    module.DefaultHttpxClient=lambda **kw:calls.append(('http',kw)) or SimpleNamespace(close=lambda:None)
    module.Anthropic=lambda **kw:calls.append(('sdk',kw)) or SimpleNamespace(close=lambda:None)
    env.setitem(sys.modules,'anthropic',module)
    AnthropicJsonClient({})._make_client()
    assert dict(os.environ)==before
    assert calls[0][1]['trust_env'] is False and calls[0][1]['follow_redirects'] is False
    assert calls[1][1]['max_retries']==0 and calls[1][1]['auth_token']=='local-test-not-a-real-token'
    assert calls[1][1]['api_key']==''


def test_provider_keeps_system_and_repair_conversation(env):
    response=sdk_response(blocks=[SimpleNamespace(type='thinking'),SimpleNamespace(type='text',text='{"x":'),SimpleNamespace(type='text',text='1}')])
    fake=FakeClient(response);client=AnthropicJsonClient({},client_factory=lambda:fake)
    msgs=[dict(role='system',content='strict JSON'),dict(role='user',content='a gate'),
          dict(role='assistant',content='{}'),dict(role='user',content='fix parent')]
    before=copy.deepcopy(msgs)
    assert client(msgs)=='{"x":1}' and fake.closed and msgs==before
    assert fake.requests[0]['system']=='strict JSON' and fake.requests[0]['messages']==msgs[1:]
    assert 'tools' not in fake.requests[0]


@pytest.mark.parametrize('stop',['max_tokens','refusal','tool_use','pause_turn','stop_sequence'])
def test_incomplete_or_action_completion_never_accepted(env,stop):
    fake=FakeClient(sdk_response(stop));client=AnthropicJsonClient({},client_factory=lambda:fake)
    with pytest.raises(ValueError):client([dict(role='user',content='JSON')])
    assert fake.closed


@pytest.mark.parametrize('base',['http://outside.invalid','https://user:secret@bad.invalid','https://bad.invalid?key=x','file:///x'])
def test_invalid_endpoint_is_not_ready(env,base):
    c=AnthropicJsonClient({'api_base':base});assert not c.readiness()['configured']


def test_protocol_is_explicit_and_failed_call_never_falls_back(env):
    env.setenv('OPENAI_API_KEY','not-real');env.setenv('RM75_LLM_MODEL','other-model');env.setenv('RM75_LLM_API_BASE','https://example.invalid/v1')
    assert isinstance(completion_client({}),DisabledCompletion)
    assert isinstance(completion_client({'provider':'anthropic'}),AnthropicJsonClient)
    from rm75_app.magnetic.llm_client import JsonChatClient
    assert isinstance(completion_client({'provider':'openai'}),JsonChatClient)
    assert isinstance(completion_client({'provider':'mock'}),DisabledCompletion)


def test_network_error_is_redacted_and_client_closed(env):
    fake=FakeClient(None)
    def fail(**kw):raise RuntimeError('secret URL and token must not leak')
    fake.stream=fail;c=AnthropicJsonClient({},client_factory=lambda:fake)
    with pytest.raises(RuntimeError) as caught:c([dict(role='user',content='JSON')])
    assert 'secret' not in str(caught.value) and fake.closed


def test_stream_deadline_is_checked_between_events(env):
    fake=FakeClient(sdk_response());c=AnthropicJsonClient({'timeout_s':1},client_factory=lambda:fake)
    values=iter([0,2]);env.setattr('rm75_app.magnetic.completion_provider.time.monotonic',lambda:next(values))
    with pytest.raises(RuntimeError):c([dict(role='user',content='JSON')])
    assert fake.closed


def test_web_readiness_and_generation_share_the_provider_factory():
    # Static callsite contract; not a browser/GPU end-to-end test.
    source=(Path(__file__).resolve().parents[1]/'rm75_app/workcell/iteration_api.py').read_text()
    tree=ast.parse(source)
    imported=[n for n in ast.walk(tree) if isinstance(n,ast.ImportFrom) and n.module=='rm75_app.magnetic.completion_provider']
    assert len(imported)==1 and any(n.name=='completion_client' for n in imported[0].names)
    c=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='IterationAPI')
    for method in ('features','_generation'):
        f=next(n for n in c.body if isinstance(n,ast.FunctionDef) and n.name==method)
        assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='completion_client' for n in ast.walk(f))
