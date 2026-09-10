"""Natural-language Jimu build design through a real LLM, validated locally.

Produces jimu_builder_scene_v1 designs that the workcell magnetic task accepts
and the original Jimu program (vendored working snapshot) executes. Provider
plumbing follows the existing orchestrator env-var convention (RM75_LLM_*) plus
an 'anthropic' provider using the official anthropic SDK. The mock provider is
deterministic and offline; it exists for tests and for exercising the
LLM -> frontend -> original-Jimu-program chain without a network call.

No design is accepted without passing rm75_app.magnetic.design.validate_design;
LLM output never bypasses the same typed contract the frontend and spec.py use.
This module never executes real robot motion.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import re
from pathlib import Path
from typing import Any

from rm75_app.magnetic.design import validate_design
from rm75_app.workcell.io import atomic_json

APP_ROOT = Path(__file__).resolve().parents[2]
# Model resolution: RM75_LLM_MODEL, then the machine's Anthropic-compatible
# model mapping (ANTHROPIC_DEFAULT_OPUS_MODEL / ANTHROPIC_MODEL), then the
# first-party default. The anthropic SDK reads ANTHROPIC_AUTH_TOKEN and
# ANTHROPIC_BASE_URL from the environment, so an Anthropic-compatible gateway
# (such as this machine's DeepSeek endpoint) works without extra configuration.
DEFAULT_MODEL = (os.environ.get('RM75_LLM_MODEL')
                 or os.environ.get('ANTHROPIC_DEFAULT_OPUS_MODEL')
                 or os.environ.get('ANTHROPIC_MODEL')
                 or 'claude-opus-5')
DEFAULT_PROVIDER = os.environ.get('RM75_LLM_PROVIDER', 'mock')
DEFAULT_PROXY = (os.environ.get('RM75_LLM_PROXY') or os.environ.get('RM75_VLM_PROXY')
                 or 'http://127.0.0.1:7897').strip()


def _make_client():
    """anthropic SDK client with a deterministic http proxy.

    The SDK rejects socks:// proxy URLs from the environment (httpx raises
    ValueError), so non-http proxy variables are hidden while the client is
    constructed; the configured http proxy (RM75_LLM_PROXY / RM75_VLM_PROXY /
    http://127.0.0.1:7897) is passed explicitly. Environment is restored.
    """
    import anthropic
    from anthropic import DefaultHttpxClient
    proxy_keys = ('http_proxy', 'https_proxy', 'HTTP_PROXY', 'HTTPS_PROXY',
                  'ALL_PROXY', 'all_proxy')
    saved = {key: os.environ.get(key) for key in proxy_keys}
    for key in proxy_keys:
        os.environ.pop(key, None)
    try:
        os.environ['http_proxy'] = DEFAULT_PROXY
        os.environ['https_proxy'] = DEFAULT_PROXY
        return anthropic.Anthropic(http_client=DefaultHttpxClient(proxy=DEFAULT_PROXY))
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

IDENTITY = dict(u=[1., 0., 0.], n=[0., 1., 0.], v=[0., 0., 1.])


def _normalize(text: str) -> str:
    return re.sub(r'\s+', ' ', (text or '').strip().lower())


def _piece(role: str, kind: str, *, locked: bool, center, parent: str | None = None) -> dict:
    piece = dict(role=role, type=kind, locked=locked, center=[float(v) for v in center],
                 u=list(IDENTITY['u']), n=list(IDENTITY['n']), v=list(IDENTITY['v']))
    if parent:
        piece['parentRole'] = parent
    return piece


def mock_design(command: str, template: dict | None = None) -> dict:
    """Deterministic offline design for a handful of fixed commands."""
    text = _normalize(command)
    if '弧' in text or 'arc' in text:
        pieces = [
            _piece('arc_base_0', 'square', locked=True, center=[0., 0., 0.]),
            _piece('arc_base_1', 'square', locked=True, center=[.074, 0., 0.]),
            _piece('arc_top_0', 'triangle', locked=False, center=[.074, .0065, 0.],
                   parent='arc_base_1'),
        ]
    else:
        pieces = [
            _piece('floor_0', 'square', locked=True, center=[0., 0., 0.]),
            _piece('floor_1', 'square', locked=True, center=[.074, 0., 0.]),
            _piece('floor_2', 'square', locked=True, center=[0., 0., .074]),
            _piece('wall_0', 'square', locked=False, center=[0., .0065, 0.],
                   parent='floor_0'),
        ]
    return {'schema': 'jimu_builder_scene_v1', 'pieces': pieces}


def _template_summary(template: dict | None) -> str:
    if not template:
        return 'No template scene is provided; generate a small self-contained design.'
    pieces = template.get('pieces', [])
    locked = [p.get('role') for p in pieces if p.get('locked')]
    movable = [p.get('role') for p in pieces if not p.get('locked')]
    return (f'Reference scene: base "{template.get("base", "")}", '
            f'plate_size_m {template.get("plate_size_m", "")}. '
            f'Locked roles: {", ".join(locked[:20]) or "none"}. '
            f'Movable roles: {", ".join(movable[:20]) or "none"}.')


def _system_prompt(template: dict | None) -> str:
    return (
        'You generate Jimu magnetic-block build designs for a real RM75 robot '
        'workcell. Output must be a single JSON object matching '
        '"jimu_builder_scene_v1": {"schema": "jimu_builder_scene_v1", "pieces": [...]}.\n'
        'Builder frame: Y-up, matrices are columns [u, n, v], units are meters. '
        'Piece dims: square (.074, .0065, .074), half_square (.037, .0065, .074), '
        'triangle (.074, .0065, .135).\n'
        'Each piece: "role" (unique string <=100 chars, [A-Za-z0-9_-]), "type" in '
        '{square, half_square, triangle}, "locked" boolean (fixed base/support pieces '
        'are locked), "center" [x,y,z], "u"/"n"/"v" orthonormal 3-vectors (default '
        '[1,0,0]/[0,1,0]/[0,0,1]), optional "parentRole" naming a parent piece.\n'
        'Constraints: 1..64 pieces, 1..12 non-locked (movable) pieces; no duplicate '
        'roles; no coincident centers; the parent graph must be acyclic; a movable '
        'piece without a parent is tolerated but discouraged. Locked floor pieces '
        'sit at y=0; stacked pieces sit above their parent (positive local y is '
        'the thin axis). Output ONLY the JSON object, no prose.'
    )


def _user_text(command: str, template: dict | None) -> str:
    return (
        f'Build command: {command}\n'
        f'{_template_summary(template)}\n'
        'Produce the design JSON now.'
    )


def _parse_json_object(text: str) -> dict:
    text = (text or '').strip()
    start = min((i for i in (text.find('{'), text.find('[')) if i >= 0), default=-1)
    end = max(text.rfind('}'), text.rfind(']'))
    if start < 0 or end < start:
        raise ValueError('No JSON object in LLM response')
    return json.loads(text[start:end + 1])


def call_anthropic(command: str, *, model: str, template: dict | None, out_dir: Path,
                   client=None, max_attempts: int = 2) -> tuple[dict, dict]:
    """Call the real Anthropic API, validate the design, repair once on error."""
    client = client or _make_client()
    system = _system_prompt(template)
    info = dict(provider='anthropic', model=model, command=command,
                api_base=os.environ.get('ANTHROPIC_BASE_URL', ''),
                proxy=DEFAULT_PROXY,
                system_sha256=hashlib.sha256(system.encode()).hexdigest(),
                attempts=[], api_key_used=False, execute_real=False,
                note='anthropic SDK resolves ANTHROPIC_AUTH_TOKEN/ANTHROPIC_BASE_URL; '
                     'credentials never enter request evidence')
    messages = [{'role': 'user', 'content': _user_text(command, template)}]
    design = None
    for attempt in range(max_attempts):
        # Streamed: thinking models can consume a large share of the token
        # budget before the first output token; streaming also avoids the
        # non-streaming timeout guard at high max_tokens.
        with client.messages.stream(model=model, max_tokens=32000,
                                    system=system, messages=messages) as stream:
            response = stream.get_final_message()
        row = dict(model=response.model, stop_reason=response.stop_reason,
                   request_id=getattr(response, '_request_id', None),
                   input_tokens=getattr(response.usage, 'input_tokens', None),
                   output_tokens=getattr(response.usage, 'output_tokens', None))
        info['attempts'].append(row)
        if response.stop_reason == 'refusal':
            details = getattr(response, 'stop_details', None)
            row['refusal_category'] = getattr(details, 'category', None)
            raise RuntimeError(f'LLM refused the request: {row}')
        if response.stop_reason == 'max_tokens':
            raise RuntimeError('LLM output truncated (max_tokens)')
        text = next((block.text for block in response.content if block.type == 'text'), '')
        try:
            payload = _parse_json_object(text)
            design = validate_design(payload).payload
            break
        except (ValueError, json.JSONDecodeError) as exc:
            messages.append({'role': 'assistant', 'content': text})
            messages.append({'role': 'user',
                             'content': f'Design rejected by local validation: {exc}. '
                                        'Return the corrected JSON object only.'})
            row['validation_error'] = str(exc)
    if design is None:
        raise RuntimeError('LLM did not produce a valid design within attempts')
    atomic_json(out_dir / 'llm_request.json', info)
    atomic_json(out_dir / 'design.json', design)
    return design, info


def load_design(path: Path) -> dict:
    payload = json.loads(Path(path).read_text())
    return validate_design(payload).payload


def submit_preview(design: dict, machine_profile: Path, out_dir: Path) -> dict:
    import time
    from rm75_app.workcell.service import WorkcellService
    service = WorkcellService(APP_ROOT, machine_profile, allow_real=False)
    try:
        spec = dict(task='magnetic', mode='preview', parameters=dict(design=copy.deepcopy(design)))
        job_id = service.submit(spec)['job_id']
        deadline = time.monotonic() + 30.
        result = service.job(job_id)
        while 'result' not in result and time.monotonic() < deadline:
            time.sleep(.05)
            result = service.job(job_id)
        report = dict(job_id=job_id, request=spec,
                      preview_result=result.get('result'),
                      status=result.get('result', {}).get('status'),
                      execute_real=False, hardware_connected=False)
        atomic_json(out_dir / 'preview_result.json', report)
        return report
    finally:
        service.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--command', help='Natural-language build command')
    parser.add_argument('--llm-provider', choices=('mock', 'anthropic'),
                        default=DEFAULT_PROVIDER)
    parser.add_argument('--llm-model', default=DEFAULT_MODEL)
    parser.add_argument('--template', type=Path,
                        help='Optional jimu_builder_scene_v1 reference scene (read-only)')
    parser.add_argument('--design-file', type=Path,
                        help='Skip the LLM; validate an existing design JSON')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--submit-preview', action='store_true')
    parser.add_argument('--machine-profile', type=Path,
                        default=APP_ROOT / 'examples/workcell/machine.example.json')
    args = parser.parse_args(argv)
    if args.design_file is None and not args.command:
        parser.error('--command is required unless --design-file is given')
    out_dir = args.output.resolve()
    out_dir.mkdir(parents=True, exist_ok=False)
    template = None
    if args.template is not None:
        template = json.loads(args.template.read_text())
    if args.design_file is not None:
        design = load_design(args.design_file)
        info = dict(provider='none', design_file=str(args.design_file.resolve()))
    elif args.llm_provider == 'mock':
        design = mock_design(args.command, template)
        validate_design(design)
        info = dict(provider='mock', model=None, command=args.command,
                    execute_real=False, note='deterministic offline design; not LLM output')
        atomic_json(out_dir / 'llm_request.json', info)
        atomic_json(out_dir / 'design.json', design)
    else:
        design, info = call_anthropic(args.command, model=args.llm_model,
                                      template=template, out_dir=out_dir)
    summary = dict(provider=args.llm_provider if args.design_file is None else 'design-file',
                   design_sha256=hashlib.sha256(
                       json.dumps(design, sort_keys=True).encode()).hexdigest(),
                   piece_count=len(design['pieces']),
                   movable_count=sum(1 for p in design['pieces'] if not p.get('locked')),
                   execute_real=False, hardware_connected=False)
    if args.submit_preview:
        preview = submit_preview(design, args.machine_profile, out_dir)
        summary['preview'] = dict(job_id=preview.get('job_id'), status=preview.get('status'),
                                  verification=preview.get('preview_result', {}).get('verification'))
    atomic_json(out_dir / 'summary.json', summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
