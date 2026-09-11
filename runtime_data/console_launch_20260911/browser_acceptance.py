"""Local acceptance driver for the operator console (R2 machine config).

Real Chromium via Playwright against a real loopback WSGI console server.
Drives the actual UI (clicks), records the console API traffic and stores
screenshots. Never submits real mode; SIM is opt-in per stage.

Usage: browser_acceptance.py <stage> <base-url> <output-dir>
Stages: preview | template | stop | session
"""
from __future__ import annotations
import json
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright, expect

CHROME = '/home/zhangzhao/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome'


def main():
    stage, base, out = sys.argv[1], sys.argv[2].rstrip('/'), Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    report = {'stage': stage, 'base': base, 'checks': [], 'api': [], 'page_errors': [],
              'console_errors': [], 'screenshots': [], 'status': 'RUNNING'}
    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=True, executable_path=CHROME)
        context = browser.new_context(viewport={'width': 1560, 'height': 1080})
        page = context.new_page()
        page.on('pageerror', lambda e: report['page_errors'].append(str(e)))
        def on_console(m):
            # The server serves no favicon; that 404 is not an application error.
            if m.type == 'error' and 'favicon' not in json.dumps(m.location):
                report['console_errors'].append({'text': m.text, 'location': dict(m.location)})
        page.on('console', on_console)

        def note(name, ok, detail=''):
            report['checks'].append({'check': name, 'ok': bool(ok), 'detail': str(detail)[:400]})
            print(('PASS ' if ok else 'FAIL ') + name + (' :: ' + str(detail)[:200] if detail else ''),
                  flush=True)

        def snap(name):
            path = out / f'{name}.png'
            page.screenshot(path=str(path), full_page=True)
            report['screenshots'].append(str(path))

        def on_response(resp):
            if '/api/workcell/console/' in resp.url:
                try:
                    body = resp.json()
                except Exception:
                    body = None
                row = {'url': resp.url.split('/api/workcell/console/')[-1], 'status': resp.status}
                if isinstance(body, dict):
                    if 'request_id' in body:
                        row['request_id'] = body['request_id']
                    if 'job_id' in body:
                        row['job_id'] = body['job_id']
                    if 'request_digest' in body:
                        row['request_digest'] = body['request_digest'][:24]
                    if 'canonical_request_digest' in body:
                        row['canonical_request_digest'] = body['canonical_request_digest'][:24]
                    if 'valid' in body:
                        row['valid'] = body['valid']
                    if 'status' in body:
                        row['status_field'] = body['status']
                report['api'].append(row)

        page.on('response', on_response)
        bad = []
        page.on('response', lambda r: bad.append({'status': r.status, 'url': r.url})
                if r.status >= 400 else None)
        report['bad_responses'] = bad
        page.goto(base + '/workcell/console/', wait_until='networkidle')
        expect(page.locator('#connection')).to_contain_text('服务器已连接')
        boot = page.evaluate("async()=>await (await fetch('/api/workcell/console/bootstrap')).json()")
        report['profile'] = boot['profile_name']
        report['version'] = boot['version']
        note('console bootstrap over real HTTP', True, boot['profile_name'])

        if stage == 'preview':
            # Multi-object preview through the real isolated worker.
            for value in ('bi', 'tennis'):
                page.locator(f'#object-grid input[value="{value}"]').check()
            order = page.locator('#object-order').inner_text()
            note('multi-select order rendered', '笔' in order and '网球' in order, order.replace('\n', ' '))
            page.click('#preflight')
            expect(page.locator('#preflight-result')).to_be_visible()
            note('preflight rendered', True, page.locator('#preflight-result h3').inner_text())
            snap('preview_preflight')
            page.click('#preview')
            expect(page.locator('#job-state')).to_contain_text('预览完成', timeout=180000)
            job_id = page.locator('#job-id').inner_text()
            raw = json.loads(page.locator('#raw-result').text_content())
            note('multi-object preview completed', True, job_id)
            note('preview result is not a motion success',
                 raw['result'].get('verification') == 'preview_only' and raw['result'].get('task_success') is None,
                 json.dumps(raw['result'].get('verification')))
            report['preview'] = {'job_id': job_id, 'request': raw['request'], 'result': raw['result']}
            snap('preview_done')

        elif stage == 'template':
            page.click('[data-task=magnetic]')
            expect(page.locator('#load-template')).to_be_enabled(timeout=30000)
            page.click('#load-template')
            expect(page.locator('#design-summary')).to_contain_text('原模板', timeout=60000)
            summary = page.locator('#design-summary').inner_text()
            note('original template loaded without LLM', '未调用 LLM' in summary, summary)
            pieces = page.locator('#piece-list span').count()
            note('template pieces rendered', pieces > 0, f'{pieces} movable pieces')
            page.click('#preflight')
            expect(page.locator('#preflight-result')).to_be_visible(timeout=60000)
            note('template preflight', True, page.locator('#preflight-result h3').inner_text())
            snap('template_loaded')

        elif stage == 'stop':
            # A long-enough CPU-only job for the shared Stop button: the PushT
            # surrogate backend, no physics backend, no GPU, no robot request.
            page.click('[data-task=pusht]')
            page.select_option('#pt-backend', 'surrogate')
            page.fill('#pt-x', '0.35'); page.fill('#pt-y', '-0.18'); page.fill('#pt-yaw', '0')
            page.fill('#pt-gx', '0.45'); page.fill('#pt-gy', '-0.10'); page.fill('#pt-gyaw', '0')
            # The form disables the step cap while "未到位则继续" is checked, so
            # this stage runs one bounded plan-push-observe loop instead.
            page.uncheck('#pt-continuous')
            expect(page.locator('#pt-steps')).to_be_enabled(timeout=10000)
            page.fill('#pt-steps', '60')
            page.click('#simulate')
            expect(page.locator('#confirm-dialog')).to_be_visible(timeout=30000)
            page.click('#confirm-dialog [value=run]')
            expect(page.locator('#stop')).to_be_enabled(timeout=300000)
            note('job running and Stop offered', True, page.locator('#job-state').inner_text())
            page.click('#stop')
            page.wait_for_function(
                "()=>{try{const v=JSON.parse(document.querySelector('#raw-result').textContent);"
                "return !!(v&&v.result);}catch(e){return false;}}", timeout=300000)
            raw = json.loads(page.locator('#raw-result').text_content())
            result = raw['result']
            note('stop reached a terminal state', result.get('status') in ('cancelled', 'failed', 'succeeded'),
                 json.dumps({'status': result.get('status'), 'task_success': result.get('task_success'),
                             'steps': result.get('steps'), 'job': raw.get('job_id'),
                             'label': page.locator('#job-state').inner_text()}))
            note('stop did not report a motion success', result.get('task_success') is not True,
                 json.dumps(result.get('task_success')))
            report['stop'] = {'job_id': raw.get('job_id'), 'result': result}
            snap('stop')

        elif stage == 'session':
            # The one known-good PushT intervention case, verbatim from
            # runtime_data/three_scene/pusht_r2_20260911/confirm_intervention_h:
            # initial [0.35,-0.18,0], goal [0.38,-0.18,0], full_arm_physics,
            # max_steps 12, speed 0.015, run_until_goal, relocate to
            # [0.358,-0.186,0.09 rad]. No new random GPU suite is started.
            page.click('[data-task=pusht]')
            expect(page.locator('#pt-continuous')).to_be_checked(timeout=30000)
            page.select_option('#pt-backend', 'full_arm_physics')
            page.select_option('#pt-geometry', 'original')
            page.fill('#pt-x', '0.35'); page.fill('#pt-y', '-0.18'); page.fill('#pt-yaw', '0')
            page.fill('#pt-gx', '0.38'); page.fill('#pt-gy', '-0.18'); page.fill('#pt-gyaw', '0')
            page.fill('#pt-speed', '0.015')
            # The form disables the step cap while "未到位则继续" is checked; set the
            # cap first, then re-arm continuous mode so the submitted request carries
            # the recorded max_steps=12 together with run_until_goal=True.
            page.uncheck('#pt-continuous')
            expect(page.locator('#pt-steps')).to_be_enabled(timeout=10000)
            page.fill('#pt-steps', '12')
            page.check('#pt-continuous')
            expect(page.locator('#pt-steps')).to_be_disabled(timeout=10000)
            page.click('#preflight')
            expect(page.locator('#preflight-result')).to_be_visible(timeout=60000)
            note('pusht preflight', True, page.locator('#preflight-result h3').inner_text())
            snap('session_preflight')
            page.click('#simulate')
            dialog = page.locator('#confirm-dialog')
            expect(dialog).to_be_visible(timeout=30000)
            note('SIM confirmation dialog shown', True, page.locator('#confirm-text').inner_text()[:120])
            snap('session_confirm')
            page.click('#confirm-dialog [value=run]')
            expect(page.locator('#pause')).to_be_enabled(timeout=900000)
            report['session_job'] = page.locator('#job-id').inner_text()
            note('session running and pause offered', True, report['session_job'])
            # Pause/resume/relocate live in the collapsed "运行中移动 T（仅仿真）"
            # panel, so open it the way an operator would before clicking.
            page.click('#intervention-panel > summary')
            expect(page.locator('#pause')).to_be_visible(timeout=30000)
            note('intervention panel opened from the UI', True,
                 page.locator('#intervention-panel > summary').inner_text())
            page.click('#pause')
            expect(page.locator('#control-status')).to_contain_text('已排队', timeout=60000)
            expect(page.locator('#relocate')).to_be_enabled(timeout=900000)
            note('pause acknowledged; relocation allowed', True,
                 page.locator('#control-status').inner_text())
            page.fill('#move-x', '0.358'); page.fill('#move-y', '-0.186')
            page.fill('#move-yaw', '5.15662')      # 0.09 rad, exactly as recorded
            page.click('#relocate')
            expect(page.locator('#control-status')).to_contain_text('已排队', timeout=60000)
            note('relocation queued', True, page.locator('#control-status').inner_text())
            expect(page.locator('#resume')).to_be_enabled(timeout=900000)
            page.click('#resume')
            page.wait_for_function(
                "()=>{try{const v=JSON.parse(document.querySelector('#raw-result').textContent);"
                "return !!(v&&v.result);}catch(e){return false;}}", timeout=1800000)
            raw = json.loads(page.locator('#raw-result').text_content())
            result = raw['result']
            note('session finished on the same request', raw['request']['parameters'].get('run_until_goal') is True
                 and raw['request']['parameters'].get('max_steps') == 12,
                 json.dumps({'job': raw.get('job_id'), 'status': result.get('status'),
                             'task_success': result.get('task_success'),
                             'steps': result.get('steps'),
                             'sent_parameters': {k: v for k, v in raw['request']['parameters'].items()
                                                 if k != 'design'},
                             'external_scene_epochs': result.get('external_scene_epochs'),
                             'verification': result.get('verification'),
                             'final_pose': (result.get('final_observation') or {}).get('pose')}))
            note('the relocated scene was actually used', (result.get('external_scene_epochs') or 0) >= 1,
                 json.dumps(result.get('external_scene_epochs')))
            report['session_result'] = result
            snap('session_done')
        else:
            raise SystemExit(f'unknown stage {stage}')

        note('no page exceptions', not report['page_errors'], report['page_errors'][:2])
        note('no browser console errors', not report['console_errors'], report['console_errors'][:2])
        report['status'] = 'PASS' if all(c['ok'] for c in report['checks']) else 'FAIL'
        browser.close()
    (out / f'report_{stage}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({'stage': stage, 'status': report['status']}), flush=True)
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
