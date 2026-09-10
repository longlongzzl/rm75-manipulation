"""One completion boundary for the production Jimu generation API.

The provider generates the existing five-field selection proposal, not robot
commands or freely invented fixed supports. Credentials stay in the web process.
No imports of robot modules or SDK/network calls occur during provider selection.
"""
from __future__ import annotations
import importlib.util
import os
import re
import time
from urllib.parse import urlsplit

from rm75_app.workcell.io import dumps, finite, integer


def _env_name(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,100}', value):
        raise ValueError('Credentials must be referenced by environment-variable name')
    return value


def _endpoint(value):
    if not isinstance(value, str) or not value:
        raise ValueError('Configure the existing Anthropic-compatible API base')
    parsed = urlsplit(value)
    if (not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment
            or not (parsed.scheme == 'https' or parsed.scheme == 'http' and
                    parsed.hostname in ('127.0.0.1', 'localhost', '::1'))):
        raise ValueError('Endpoint must use HTTPS or explicitly configured loopback HTTP')
    return value.rstrip('/')


class AnthropicJsonClient:
    """Official SDK adapter, also usable with the user's configured compatible API.

    Uses SDK re-exports rather than constructing version-specific httpx objects.
    No mutation of process-wide proxy variables; no automatic SDK retries.
    """
    def __init__(self, settings=None, *, client_factory=None):
        self.settings = dict(settings or {})
        allowed = {'model', 'api_base', 'api_key_env', 'auth_token_env', 'proxy',
                   'timeout_s', 'max_tokens', 'enabled'}
        if set(self.settings) - allowed:
            raise ValueError('Unknown Anthropic-compatible Jimu settings')
        if type(self.settings.get('enabled', True)) is not bool:
            raise ValueError('enabled must be boolean')
        if self.settings.get('api_key_env') and self.settings.get('auth_token_env'):
            raise ValueError('Choose either API-key or bearer-token authentication')
        self.model = (self.settings.get('model') or os.environ.get('RM75_LLM_MODEL')
                      or os.environ.get('ANTHROPIC_DEFAULT_OPUS_MODEL') or os.environ.get('ANTHROPIC_MODEL'))
        self.base = self.settings.get('api_base') or os.environ.get('ANTHROPIC_BASE_URL', '')
        explicit_key = self.settings.get('api_key_env')
        explicit_token = self.settings.get('auth_token_env')
        token_default = bool(os.environ.get('ANTHROPIC_AUTH_TOKEN')) and not explicit_key
        self.auth_kind = 'auth_token' if explicit_token or token_default else 'api_key'
        self.key_env = _env_name(explicit_token or explicit_key or
                                 ('ANTHROPIC_AUTH_TOKEN' if token_default else 'ANTHROPIC_API_KEY'))
        self.proxy = self.settings.get('proxy', os.environ.get('RM75_LLM_PROXY',
                                              os.environ.get('RM75_VLM_PROXY', '')))
        if not isinstance(self.proxy, str):
            raise ValueError('proxy must be a server-configured string')
        self.timeout = finite(self.settings.get('timeout_s', 60), 'timeout_s', 1, 120)
        self.tokens = integer(self.settings.get('max_tokens', 4096), 'max_tokens', 128, 32768)
        self.client_factory = client_factory

    def readiness(self):
        missing = []
        if not isinstance(self.model, str) or not self.model.strip(): missing.append('model')
        try: _endpoint(self.base)
        except ValueError: missing.append('valid api_base')
        if not os.environ.get(self.key_env): missing.append('credential environment value')
        if self.client_factory is None and importlib.util.find_spec('anthropic') is None:
            missing.append('anthropic SDK in web interpreter')
        return dict(configured=not missing and self.settings.get('enabled', True), missing=missing,
                    provider='anthropic_compatible_json', makes_robot_calls=False)

    def _make_client(self):
        import anthropic
        kwargs = dict(base_url=_endpoint(self.base), timeout=self.timeout, max_retries=0)
        # Empty unused credential prevents SDK environment auto-discovery from
        # sending a second secret to a server that only uses bearer authentication.
        kwargs.update(api_key='', auth_token='')
        kwargs[self.auth_kind] = os.environ[self.key_env]
        http = anthropic.DefaultHttpxClient(proxy=self.proxy or None, trust_env=False,
                                            follow_redirects=False, timeout=self.timeout)
        try:
            return anthropic.Anthropic(http_client=http, **kwargs)
        except BaseException:
            http.close()
            raise

    def __call__(self, messages):
        if not self.readiness()['configured']:
            raise ValueError('Configure the existing Anthropic-compatible endpoint/model/credential')
        if not isinstance(messages, list) or not messages:
            raise ValueError('Expected a nonempty design message list')
        system = []
        turns = []
        for row in messages:
            if not isinstance(row, dict) or set(row) != {'role', 'content'} or not isinstance(row['content'], str):
                raise ValueError('Design messages must contain text role/content only')
            role = row['role']
            if role == 'system' and not turns: system.append(row['content'])
            elif role in ('user', 'assistant'): turns.append(dict(row))
            else: raise ValueError('Unsupported role or late system message')
        if not turns or turns[0]['role'] != 'user':
            raise ValueError('Anthropic design conversation must start with a user turn')
        request = dict(model=self.model, system='\n'.join(system), messages=turns, max_tokens=self.tokens)
        if len(dumps(request).encode()) > 180_000:
            raise ValueError('Design request exceeds bounded payload size')
        client = None
        try:
            client = self.client_factory() if self.client_factory else self._make_client()
            start = time.monotonic()
            with client.messages.stream(**request) as stream:
                # Socket/read timeout and event-boundary wall budget; not a
                # claim that an arbitrary external SDK call is hard-preemptible.
                for _ in stream:
                    if time.monotonic() - start > self.timeout:
                        raise TimeoutError('Design stream wall budget exhausted')
                response = stream.get_final_message()
        except Exception:
            raise RuntimeError('Anthropic-compatible design request failed or timed out') from None
        finally:
            if client is not None:
                try: client.close()
                except Exception: pass
        if response.stop_reason != 'end_turn':
            raise ValueError('Design completion was not a complete end_turn')
        content = []
        for block in response.content:
            if block.type == 'text': content.append(block.text)
            elif block.type not in ('thinking', 'redacted_thinking'):
                raise ValueError('Design completion must not contain tools or executable actions')
        text = ''.join(content)
        if not text.strip() or len(text) > 12000:
            raise ValueError('Design JSON response is empty or exceeds size budget')
        return text


class DisabledCompletion:
    def readiness(self):
        return dict(configured=False, missing=['explicit real provider or unambiguous configured credentials'],
                    provider='disabled_or_ambiguous', makes_robot_calls=False)
    def __call__(self, messages):
        raise ValueError('Production LLM generation cannot substitute an offline mock result')


def completion_client(settings=None):
    """Explicit provider wins; auto selects only a single configured protocol.

    A failed call is never retried through another provider. Existing OpenAI
    settings and tests remain supported without installing Anthropic locally.
    """
    data = dict(settings or {})
    provider = data.pop('provider', os.environ.get('RM75_LLM_PROVIDER', 'auto'))
    if provider in ('anthropic', 'anthropic_compatible'):
        return AnthropicJsonClient(data)
    from .llm_client import JsonChatClient
    if provider in ('openai', 'openai_compatible', 'deepseek'):
        return JsonChatClient(data)
    if provider == 'mock': return DisabledCompletion()
    if provider != 'auto': raise ValueError('Unknown Jimu completion provider')
    # Keep existing OpenAI-shaped explicit settings authoritative in auto mode.
    if data.get('api_base') or data.get('api_key_env') or 'token_parameter' in data or 'json_mode' in data:
        return JsonChatClient(data)
    if data.get('auth_token_env'):
        return AnthropicJsonClient(data)
    openai = JsonChatClient(data)
    anthropic = AnthropicJsonClient(data)
    have_openai = openai.readiness()['configured']
    have_anthropic = anthropic.readiness()['configured']
    if have_openai and have_anthropic: return DisabledCompletion()
    return anthropic if have_anthropic else openai
