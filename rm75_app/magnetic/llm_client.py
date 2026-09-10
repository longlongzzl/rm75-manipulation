"""Small OpenAI-compatible JSON transport, with server-only credentials.

Uses the existing RM75_LLM_* environment convention without importing the large
robot orchestrator. No API call occurs on import, construction, or preview.
"""
from __future__ import annotations
import os
import re
import urllib.request
import urllib.error
from urllib.parse import urlsplit
from rm75_app.workcell.io import dumps, loads, finite, integer


class JsonChatClient:
    def __init__(self, settings=None, *, opener=None):
        settings = dict(settings or {})
        allowed = {'model', 'api_base', 'api_key_env', 'proxy', 'timeout_s', 'max_tokens',
                   'token_parameter', 'json_mode', 'enabled'}
        if set(settings) - allowed: raise ValueError('Unknown Jimu LLM setting')
        self.settings = settings
        self.model = settings.get('model') or os.environ.get('RM75_LLM_MODEL', '')
        base = settings.get('api_base') or os.environ.get('RM75_LLM_API_BASE', '')
        self.key_env = settings.get('api_key_env') or os.environ.get('RM75_LLM_API_KEY_ENV', 'OPENAI_API_KEY')
        self.url = base.rstrip('/') + '/chat/completions' if base else ''
        self.timeout = finite(settings.get('timeout_s', 30), 'LLM timeout_s', 1, 60)
        self.tokens = integer(settings.get('max_tokens', 1600), 'LLM max_tokens', 128, 4096)
        self.token_parameter = settings.get('token_parameter', 'max_completion_tokens')
        if self.token_parameter not in ('max_tokens', 'max_completion_tokens'):
            raise ValueError('Invalid completion-token parameter')
        self.json_mode = settings.get('json_mode', True)
        if type(self.json_mode) is not bool: raise ValueError('json_mode must be boolean')
        if not isinstance(self.key_env, str) or not re.fullmatch('[A-Za-z_][A-Za-z0-9_]{0,100}', self.key_env):
            raise ValueError('api_key_env must name an environment variable, not contain a key')
        self.opener = opener
        self.proxy = settings.get('proxy', os.environ.get('RM75_LLM_PROXY', ''))

    def readiness(self):
        missing = []
        if not isinstance(self.model, str) or not self.model.strip(): missing.append('model')
        if not self.url: missing.append('api_base')
        if not os.environ.get(self.key_env): missing.append('api_key_env value')
        return dict(configured=not missing and self.settings.get('enabled', True) is True,
                    missing=missing, provider='openai_compatible_json', makes_robot_calls=False)

    def __call__(self, messages):
        if not self.readiness()['configured']:
            raise ValueError('Jimu LLM is not configured; set the server RM75_LLM_* variables/profile')
        parsed = urlsplit(self.url)
        if (parsed.username or parsed.password or parsed.query or parsed.fragment or not parsed.hostname
                or (parsed.scheme != 'https' and not (parsed.scheme == 'http' and parsed.hostname in ('127.0.0.1', 'localhost', '::1')))):
            raise ValueError('LLM API requires HTTPS, or an explicitly configured loopback endpoint')
        payload = dict(model=self.model, messages=messages, **{self.token_parameter: self.tokens})
        if self.json_mode: payload['response_format'] = {'type': 'json_object'}
        raw = dumps(payload).encode()
        if len(raw) > 180_000: raise ValueError('LLM request exceeds design-generation budget')
        request = urllib.request.Request(self.url, data=raw, method='POST', headers={
            'Content-Type': 'application/json', 'Authorization': 'Bearer ' + os.environ[self.key_env]})
        # Refuse redirects: do not forward authorization to another destination.
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *args, **kwargs): return None
        opener = self.opener or urllib.request.build_opener(
            urllib.request.ProxyHandler({'https': self.proxy, 'http': self.proxy} if self.proxy else {}), NoRedirect())
        try:
            with opener.open(request, timeout=self.timeout) as response:
                data = response.read(524289)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f'LLM HTTP {exc.code}; no design accepted') from None
        except (OSError, urllib.error.URLError):
            raise RuntimeError('LLM transport failed or timed out; no design accepted') from None
        if len(data) > 524288: raise ValueError('LLM response exceeds size budget')
        value = loads(data.decode('utf-8'))
        choices = value.get('choices')
        if not isinstance(choices, list) or len(choices) != 1:
            raise ValueError('LLM returned no unique completion')
        choice = choices[0]; message = choice.get('message', {})
        if choice.get('finish_reason') != 'stop' or message.get('refusal'):
            raise ValueError('LLM completion was truncated/refused; no design accepted')
        content = message.get('content')
        if not isinstance(content, str) or not content.strip() or len(content) > 12000:
            raise ValueError('LLM returned invalid design JSON content')
        return content
