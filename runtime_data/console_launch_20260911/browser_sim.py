"""Same-design native SIM through the console, reusing a durable generation.

Imports the design bundle produced by an earlier generation (same design_digest,
same generation_proof) through the console's file-import path, then preflights
and runs the native Jimu worker. No model call is made here.

Usage: browser_sim.py <bundle.json> <base-url> <output-dir>
"""
from __future__ import annotations
import json
import subprocess
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright, expect

CHROME = '/home/zhangzhao/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome'
GPU_WAIT_S = 1800


def gpu_others():
    try:
        out = subprocess.run(['nvidia-smi', '--query-compute-apps=pid,used_memory',
                              '--format=csv,noheader'], capture_output=True, text=True, timeout=20).stdout
    except Exception:
        return []
    return [line.strip() for line in out.splitlines() if line.strip()]


def main():
    bundle_path, base, out = Path(sys.argv[1]), sys.argv[2].rstrip('/'), Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    bundle = json.loads(bundle_path.read_text())
    digest = bundle['generation_proof']['design_digest']
    report = {'stage': 'sim', 'bundle': str(bundle_path), 'design_digest': digest,
              'checks': [], 'page_errors': [], 'console_errors': [], 'screenshots': [],
              'status': 'RUNNING'}
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True, executable_path=CHROME)
            page = browser.new_context(viewport={'width': 1560, 'height': 1080}).new_page()
            page.on('pageerror', lambda e: report['page_errors'].append(str(e)))
            page.on('console', lambda m: report['console_errors'].append({'text': m.text})
                    if m.type == 'error' and 'favicon' not in json.dumps(m.location) else None)

            def note(name, ok, detail=''):
                report['checks'].append({'check': name, 'ok': bool(ok), 'detail': str(detail)[:600]})
                print(('PASS ' if ok else 'FAIL ') + name +
                      (' :: ' + str(detail)[:240] if detail else ''), flush=True)

            def snap(name):
                path = out / f'{name}.png'
                page.screenshot(path=str(path), full_page=True)
                report['screenshots'].append(str(path))

            page.goto(base + '/workcell/console/', wait_until='networkidle')
            expect(page.locator('#connection')).to_contain_text('服务器已连接')
            page.click('[data-task=magnetic]')
            expect(page.locator('#load-template')).to_be_enabled(timeout=30000)
            page.set_input_files('#design-file', str(bundle_path))
            expect(page.locator('#design-summary')).to_contain_text('导入设计', timeout=60000)
            note('bundle imported with its provenance', digest[:16] in page.locator('#design-summary').inner_text()
                 or '来源已校验' in page.locator('#design-summary').inner_text(),
                 page.locator('#design-summary').inner_text())
            snap('sim_imported')
            page.click('#preflight')
            expect(page.locator('#preflight-result')).to_be_visible(timeout=60000)
            note('preflight passed', '配置与请求检查通过' in page.locator('#preflight-result h3').inner_text(),
                 page.locator('#preflight-result h3').inner_text())

            others = gpu_others()
            waited = 0
            while others and waited < GPU_WAIT_S:
                if waited % 300 == 0:
                    print(f'waiting for GPU: {others} (waited {waited}s)', flush=True)
                time.sleep(20); waited += 20
                others = gpu_others()
            note('GPU was serialized for the native SIM', not others,
                 json.dumps({'other_compute_apps': others, 'waited_s': waited}))

            page.click('#simulate')
            expect(page.locator('#confirm-dialog')).to_be_visible(timeout=30000)
            note('SIM confirmation names the same design', digest in page.locator('#confirm-spec').inner_text(),
                 page.locator('#confirm-spec').inner_text().replace('\n', ' ')[:200])
            page.click('#confirm-dialog [value=run]')
            page.wait_for_function(
                "()=>{try{const v=JSON.parse(document.querySelector('#raw-result').textContent);"
                "return !!(v&&v.result);}catch(e){return false;}}", timeout=2400000)
            raw = json.loads(page.locator('#raw-result').text_content())
            result = raw['result']
            job_proof = (raw['request'].get('parameters') or {}).get('generation_proof') or {}
            note('native SIM ran the identical proof', job_proof.get('design_digest') == digest,
                 json.dumps({'job_digest': job_proof.get('design_digest'), 'expected': digest}))
            report['job_id'] = raw.get('job_id')
            report['job_state'] = page.locator('#job-state').inner_text()
            report['result'] = result
            report['request_parameters'] = {k: v for k, v in raw['request'].get('parameters', {}).items()
                                            if k != 'design'}
            note('native SIM terminal state', True, json.dumps({k: result.get(k) for k in
                 ('status', 'task_success', 'generated_design_used_unchanged', 'mode', 'task',
                  'hardware_connected', 'native_completed_cycles', 'error')}, ensure_ascii=False))
            note('the run reported no real-robot execution',
                 result.get('execute_real', False) is False and raw['request'].get('mode') == 'sim',
                 json.dumps({'execute_real': result.get('execute_real'), 'mode': raw['request'].get('mode'),
                             'hardware_connected': result.get('hardware_connected')}))
            snap('sim_done')
            note('no page exceptions', not report['page_errors'], report['page_errors'][:2])
            note('no browser console errors (favicon excluded)', not report['console_errors'],
                 report['console_errors'][:2])
            report['status'] = 'PASS' if all(c['ok'] for c in report['checks']) else 'FAIL'
    except Exception as exc:
        report['status'] = 'FAIL'
        report['exception'] = f'{type(exc).__name__}: {exc}'[:800]
        report['checks'].append({'check': 'driver completed', 'ok': False, 'detail': report['exception']})
    finally:
        (out / 'report_jimu_sim.json').write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                                  encoding='utf-8')
        print(json.dumps({'stage': 'sim', 'status': report['status']}), flush=True)
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
