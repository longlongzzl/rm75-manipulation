import io
import json
import urllib.error
from types import SimpleNamespace
import pytest
from rm75_app.magnetic.llm_client import JsonChatClient


def configured(monkeypatch,output):
    monkeypatch.setenv('TEST_LLM_SECRET','do-not-log-this-secret')
    captured=[]
    class Opener:
        def open(self,request,timeout):
            captured.append((request,timeout));return io.BytesIO(output)
    client=JsonChatClient({'model':'configured-model','api_base':'https://example.invalid/v1',
                          'api_key_env':'TEST_LLM_SECRET'},opener=Opener())
    return client,captured


def response(finish='stop',content='{}',refusal=None):
    return json.dumps({'choices':[{'finish_reason':finish,'message':{'content':content,'refusal':refusal}}]}).encode()


def test_no_request_before_explicit_call(monkeypatch):
    client,calls=configured(monkeypatch,response())
    assert client.readiness()['configured'] and calls==[]
    assert 'do-not-log' not in json.dumps(client.readiness())
    assert client([{'role':'user','content':'a house'}])=='{}'
    request,timeout=calls[0];data=json.loads(request.data)
    assert request.full_url=='https://example.invalid/v1/chat/completions'
    assert data['response_format']=={'type':'json_object'} and timeout==30
    assert data['max_completion_tokens']==1600


@pytest.mark.parametrize('finish,content,refusal', [('length','{}',None),('content_filter','{}',None),('stop','{}','refused'),('stop','',None),('stop',None,None)])
def test_incomplete_refusal_rejected(monkeypatch,finish,content,refusal):
    client,_=configured(monkeypatch,response(finish,content,refusal))
    with pytest.raises(ValueError):client([])


@pytest.mark.parametrize('url',['http://outside.example/v1','https://secret:password@example.invalid/v1','https://example.invalid/v1?secret=abc','file:///tmp/x'])
def test_untrusted_transport_settings_rejected(monkeypatch,url):
    client,calls=configured(monkeypatch,response());client.url=url
    with pytest.raises(ValueError):client([])
    assert not calls


def test_oversized_response_rejected(monkeypatch):
    client,_=configured(monkeypatch,b'x'*524289)
    with pytest.raises(ValueError):client([])


def test_missing_key_does_not_call(monkeypatch):
    client,calls=configured(monkeypatch,response());monkeypatch.delenv('TEST_LLM_SECRET')
    with pytest.raises(ValueError):client([])
    assert not calls


def test_http_error_not_echo_secret(monkeypatch):
    client,_=configured(monkeypatch,response())
    def fail(*args,**kwargs):raise urllib.error.HTTPError('secret-url',401,'secret-body',None,None)
    client.opener=SimpleNamespace(open=fail)
    with pytest.raises(RuntimeError) as e:client([])
    assert 'secret' not in str(e.value)
