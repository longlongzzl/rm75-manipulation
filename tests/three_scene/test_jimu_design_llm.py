"""LLM -> jimu_builder_scene_v1 -> magnetic preview: offline contracts."""
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from rm75_app.llm import jimu_design_llm as jdl
from rm75_app.magnetic.design import validate_design


def test_mock_designs_are_deterministic_and_valid():
    for command in ('搭一个3x3底板的三层房子', 'arc base build', '随便搭一个'):
        design = jdl.mock_design(command)
        assert design == jdl.mock_design(command)
        validated = validate_design(design)
        assert 1 <= len(validated.ordered_roles) <= 12
        assert design['schema'] == 'jimu_builder_scene_v1'
        assert any(p.get('locked') for p in design['pieces'])


def test_system_prompt_teaches_the_typed_contract_without_secrets():
    prompt = jdl._system_prompt(None)
    assert 'jimu_builder_scene_v1' in prompt
    assert 'square' in prompt and 'triangle' in prompt and 'half_square' in prompt
    assert '1..12' in prompt and 'acyclic' in prompt
    assert 'ANTHROPIC' not in prompt and 'api_key' not in prompt.lower()


class _FakeUsage:
    input_tokens = 100
    output_tokens = 200


def _fake_response(text, stop='end_turn', model='claude-opus-5'):
    return NS(model=model, stop_reason=stop, stop_details=None,
              usage=_FakeUsage(), _request_id='req_test',
              content=[NS(type='text', text=text)])


class _FakeStream:
    def __init__(self, response):
        self._response = response

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def get_final_message(self):
        return self._response


class _FakeClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    @property
    def messages(self):
        return self

    def create(self, **kwargs):
        self.calls.append(kwargs)
        return self.responses.pop(0)

    def stream(self, **kwargs):
        self.calls.append(kwargs)
        return _FakeStream(self.responses.pop(0))


def _design_json():
    return json.dumps(jdl.mock_design('x'))


def test_anthropic_call_validates_and_repairs_once(tmp_path):
    bad = '{"schema":"jimu_builder_scene_v1","pieces":[{"role":"a","type":"square","locked":false}]}'
    client = _FakeClient([_fake_response(bad), _fake_response(_design_json())])
    design, info = jdl.call_anthropic('build it', model='claude-opus-5', template=None,
                                      out_dir=tmp_path, client=client)
    validate_design(design)
    assert len(client.calls) == 2
    assert info['attempts'][0]['validation_error']
    assert info['attempts'][1]['stop_reason'] == 'end_turn'
    assert info['api_key_used'] is False
    # Evidence on disk contains no credential material.
    evidence = json.loads((tmp_path / 'llm_request.json').read_text())
    assert 'authorization' not in json.dumps(evidence).lower()
    assert (tmp_path / 'design.json').is_file()


def test_anthropic_call_raises_on_refusal_and_truncation(tmp_path):
    client = _FakeClient([_fake_response('{}', stop='refusal')])
    with pytest.raises(RuntimeError, match='refused'):
        jdl.call_anthropic('x', model='claude-opus-5', template=None, out_dir=tmp_path, client=client)
    client = _FakeClient([_fake_response('{}', stop='max_tokens')])
    with pytest.raises(RuntimeError, match='truncated'):
        jdl.call_anthropic('x', model='claude-opus-5', template=None, out_dir=tmp_path, client=client)


def test_load_design_rejects_invalid_json(tmp_path):
    bad = tmp_path / 'bad.json'
    bad.write_text(json.dumps({'schema': 'nope'}))
    with pytest.raises(ValueError):
        jdl.load_design(bad)


def _workcell_lease_busy() -> bool:
    import fcntl
    lock = Path(__file__).resolve().parents[2] / 'runtime_data/workcell/robot.lock'
    lock.parent.mkdir(parents=True, exist_ok=True)
    stream = lock.open('a+')
    try:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            return True
        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        return False
    finally:
        stream.close()


def test_cli_mock_end_to_end_preview(tmp_path):
    if _workcell_lease_busy():
        pytest.skip('workcell lease held by an active job; preview needs the free workcell')
    profile = Path(__file__).resolve().parents[2] / 'examples/workcell/machine.example.json'
    out = tmp_path / 'out'
    code = jdl.main(['--command', '搭一个三层房子', '--llm-provider', 'mock',
                     '--output', str(out), '--submit-preview', '--machine-profile', str(profile)])
    assert code == 0
    summary = json.loads((out / 'summary.json').read_text())
    assert summary['provider'] == 'mock' and summary['execute_real'] is False
    assert summary['preview']['status'] == 'command_completed_unverified'
    assert summary['preview']['verification'] == 'preview_only'
    assert validate_design(json.loads((out / 'design.json').read_text()))


def test_template_summary_lists_roles_without_dumping_the_scene():
    template = json.loads((Path(__file__).resolve().parents[2]
        / 'rm75_app/_vendor/jimu_scenes/Beta_demo-codex-v0.9/jimu_tasks'
        / 'tag2_arc_base/builder_scene.json').read_text())
    summary = jdl._template_summary(template)
    assert 'arc_triangle_0' in summary and 'triangle_19' in summary
    assert len(summary) < 2000
