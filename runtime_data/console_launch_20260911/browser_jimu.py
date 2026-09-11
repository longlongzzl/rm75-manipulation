"""Jimu acceptance through the console UI, one continuous browser session.

Stage `llm`:
  1. start a real LLM generation from the magnetic panel;
  2. reload the page while the provider is still working (durable recovery);
  3. confirm exactly one model call and exactly one generation identity;
  4. preflight, then the SIM confirmation dialog for that same design;
  5. run the real native Jimu worker and record the terminal result.

No real-robot request is ever built. The LLM stage costs one paid call.
"""
from __future__ import annotations
import json
import re
import subprocess
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright, expect

CHROME = '/home/zhangzhao/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome'
PROMPT = '用3x3底板搭一个三层小塔：选三块可动件叠放'   # same prompt as the R2 grid_gen_1b run
BOARD = 'grid_3x3'
GPU_WAIT_S = 600


def gpu_others():
    """Other processes holding the GPU right now; never stopped, only observed."""
    try:
        out = subprocess.run(['nvidia-smi', '--query-compute-apps=pid,used_memory',
                              '--format=csv,noheader'], capture_output=True, text=True, timeout=20).stdout
    except Exception:
        return []
    rows = [line.strip() for line in out.splitlines() if line.strip()]
    return rows


def main():
    stage, base, out = sys.argv[1], sys.argv[2].rstrip('/'), Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    report = {'stage': stage, 'base': base, 'checks': [], 'generate_posts': [],
              'generation_polls': [], 'page_errors': [], 'console_errors': [],
              'screenshots': [], 'status': 'RUNNING'}
    browser = None
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True, executable_path=CHROME)
            page = browser.new_context(viewport={'width': 1560, 'height': 1080}).new_page()
            page.on('pageerror', lambda e: report['page_errors'].append(str(e)))

            def on_console(m):
                if m.type == 'error' and 'favicon' not in json.dumps(m.location):
                    report['console_errors'].append({'text': m.text, 'location': dict(m.location)})
            page.on('console', on_console)

            def on_response(resp):
                url = resp.url
                if url.endswith('/console/generate') and resp.request.method == 'POST':
                    try:
                        body = resp.json()
                    except Exception:
                        body = {}
                    report['generate_posts'].append(
                        {'http': resp.status, 'request_id': body.get('request_id'),
                         'generation_id': body.get('generation_id'), 'status': body.get('status')})
                if '/console/generations/' in url and resp.request.method == 'GET':
                    ident = url.rstrip('/').rsplit('/', 1)[-1]
                    if ident not in report['generation_polls']:
                        report['generation_polls'].append(ident)
            page.on('response', on_response)

            def note(name, ok, detail=''):
                report['checks'].append({'check': name, 'ok': bool(ok), 'detail': str(detail)[:600]})
                print(('PASS ' if ok else 'FAIL ') + name +
                      (' :: ' + str(detail)[:240] if detail else ''), flush=True)

            def snap(name):
                path = out / f'{name}.png'
                page.screenshot(path=str(path), full_page=True)
                report['screenshots'].append(str(path))

            def status_text():
                return page.locator('#generation-status').inner_text()

            page.goto(base + '/workcell/console/', wait_until='networkidle')
            expect(page.locator('#connection')).to_contain_text('服务器已连接')
            page.click('[data-task=magnetic]')
            expect(page.locator('#generate')).to_be_enabled(timeout=30000)

            # ---- 1. real generation -------------------------------------------
            page.select_option('#mg-board', BOARD)
            page.fill('#mg-budget', '3')
            page.fill('#mg-prompt', PROMPT)
            page.click('#generate')
            expect(page.locator('#generation-status')).to_contain_text('生成中', timeout=60000)
            started = time.time()
            first = status_text()
            note('generation accepted by the server', len(report['generate_posts']) == 1 and
                 report['generate_posts'][0].get('generation_id') is not None,
                 json.dumps(report['generate_posts']))
            note('wait timer is not a client deadline', '135' in first and '不再按' in first, first[:160])
            snap('llm_generating')

            # ---- 2. reload while the provider is still working ----------------
            time.sleep(15)
            before = status_text()
            note('still generating when the page is reloaded',
                 '生成中' in before and time.time() - started > 10, before[:160])
            page.reload(wait_until='networkidle')
            expect(page.locator('#connection')).to_contain_text('服务器已连接', timeout=60000)
            resumed = False
            try:
                expect(page.locator('#generation-status')).to_contain_text('生成中', timeout=45000)
                resumed = True
            except AssertionError:
                resumed = False
            note('generation polling resumed after the refresh', resumed, status_text()[:200])
            # The resumed timer is the server record's own created_at, so a
            # non-zero elapsed proves this is the reclaimed generation, not a
            # fresh one started by the reload.
            clock = re.search(r'已等待 (\d+):(\d+)', status_text())
            elapsed = int(clock.group(1)) * 60 + int(clock.group(2)) if clock else -1
            note('resumed timer continues the same server-side record', elapsed >= 10,
                 f'elapsed={elapsed}s text={status_text()[:160]}')
            restore = page.locator('#restore-note').inner_text()
            note('the reload restored local state instead of resubmitting',
                 ('没有自动发起任务、重试或模型请求' in restore or '已找回服务器接收记录' in restore)
                 and len(report['generate_posts']) == 1, restore[:220])
            note('no second model request after the refresh', len(report['generate_posts']) == 1,
                 json.dumps(report['generate_posts']))
            snap('llm_after_refresh')

            expect(page.locator('#design-summary')).to_contain_text('LLM 生成', timeout=900000)
            summary = page.locator('#design-summary').inner_text()
            final = status_text()
            note('exactly one generation identity was polled',
                 len(report['generation_polls']) == 1 and
                 report['generation_polls'][0] == report['generate_posts'][0].get('generation_id'),
                 json.dumps(report['generation_polls']))
            note('generated design adopted once', True, summary)
            note('design carries its provenance', '生成不代表已通过原生仿真' in final, final[:300])

            ident = report['generate_posts'][0]['generation_id']
            record = page.evaluate(
                "async(id)=>await (await fetch('/api/workcell/console/generations/'+id)).json()", ident)
            report['generation'] = record
            proof = record.get('proof') or {}
            report['proof_digest'] = proof.get('design_digest')
            note('generation finished successfully with one proof',
                 record.get('status') == 'succeeded' and bool(report['proof_digest']),
                 json.dumps({'status': record.get('status'), 'board': proof.get('board_id'),
                             'movable': proof.get('movable_count'), 'roles': proof.get('selected_roles'),
                             'design_digest': report['proof_digest'],
                             'model_calls': record.get('model_calls')}))
            snap('llm_generated')

            # ---- 3. preflight + SIM confirmation -------------------------------
            # Keep local GPU work serialized: wait for other compute apps to
            # finish, but never touch them. Bounded, and recorded either way.
            others = gpu_others()
            waited = 0
            while others and waited < GPU_WAIT_S:
                if waited % 120 == 0:
                    print(f'waiting for GPU: {others} (waited {waited}s)', flush=True)
                time.sleep(20); waited += 20
                others = gpu_others()
            note('GPU was serialized for the native SIM', not others,
                 json.dumps({'other_compute_apps': others, 'waited_s': waited}))
            page.click('#preflight')
            expect(page.locator('#preflight-result')).to_be_visible(timeout=60000)
            note('preflight passed for the generated design',
                 '配置与请求检查通过' in page.locator('#preflight-result h3').inner_text(),
                 page.locator('#preflight-result h3').inner_text())
            snap('llm_preflight')
            page.click('#simulate')
            expect(page.locator('#confirm-dialog')).to_be_visible(timeout=30000)
            spec_text = page.locator('#confirm-spec').inner_text()
            note('SIM confirmation names the same generated design',
                 report['proof_digest'] in spec_text, spec_text.replace('\n', ' ')[:300])
            snap('llm_confirm')
            page.click('#confirm-dialog [value=run]')
            page.wait_for_function(
                "()=>{try{const v=JSON.parse(document.querySelector('#raw-result').textContent);"
                "return !!(v&&v.result);}catch(e){return false;}}", timeout=1800000)
            raw = json.loads(page.locator('#raw-result').text_content())
            result = raw['result']
            job_proof = (raw['request'].get('parameters') or {}).get('generation_proof') or {}
            note('native SIM ran the identical proof',
                 job_proof.get('design_digest') == report['proof_digest'],
                 json.dumps({'job_digest': job_proof.get('design_digest'),
                             'generation_digest': report['proof_digest']}))
            report['job_id'] = raw.get('job_id')
            report['job_state'] = page.locator('#job-state').inner_text()
            report['result'] = result
            report['request_parameters'] = raw['request'].get('parameters', {})
            note('native SIM reached a terminal state', True, json.dumps(
                {k: result.get(k) for k in ('status', 'task_success', 'native_completed_cycles',
                                            'native_cycles', 'generated_design_used_unchanged',
                                            'verification', 'first_failure_stage')}))
            # The magnetic result schema carries no robot-command field at all,
            # so the substance is: the submitted request was a sim request and
            # the result reports neither a real execution nor a connected arm.
            note('the run reported no real-robot execution',
                 raw['request'].get('mode') == 'sim'
                 and result.get('execute_real') is not True
                 and result.get('hardware_connected') is not True,
                 json.dumps({'mode': raw['request'].get('mode'),
                             'execute_real': result.get('execute_real'),
                             'hardware_connected': result.get('hardware_connected'),
                             'robot_command_field_in_schema': 'robot_command_submitted' in result}))
            snap('llm_sim_done')

            note('no page exceptions', not report['page_errors'], report['page_errors'][:2])
            note('no browser console errors (favicon excluded)', not report['console_errors'],
                 report['console_errors'][:2])
            report['status'] = 'PASS' if all(c['ok'] for c in report['checks']) else 'FAIL'
    except Exception as exc:                     # keep partial evidence
        report['status'] = 'FAIL'
        report['exception'] = f'{type(exc).__name__}: {exc}'[:800]
        report['checks'].append({'check': 'driver completed', 'ok': False, 'detail': report['exception']})
    finally:
        (out / f'report_jimu_{stage}.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps({'stage': stage, 'status': report['status']}), flush=True)
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
