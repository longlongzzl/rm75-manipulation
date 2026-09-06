#!/usr/bin/env python3
"""Local browser regression, with normal navigation and the actual HTTP transport.

Run a non-hardware workcell server first. This script refuses a real-enabled
server and never clicks the real or arm endpoint. Playwright/Chromium required.
"""
from pathlib import Path
import argparse
import json
from urllib.parse import urlsplit
from playwright.sync_api import sync_playwright, expect


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:7861')
    parser.add_argument('--output',type=Path,default=Path('validation/local-browser'))
    parser.add_argument('--chromium',type=Path)
    parser.add_argument('--design',type=Path,help='Existing full builder JSON for import/export regression')
    args=parser.parse_args()
    origin=urlsplit(args.url)
    if origin.scheme!='http' or origin.hostname not in ('127.0.0.1','localhost'):
        raise ValueError('Only the local non-hardware test server is supported')
    args.output.mkdir(parents=True,exist_ok=True)
    report={'checks':[],'errors':[],'transport':'normal browser navigation and HTTP',
            'robot_connected':False,'hardware_actions_requested':False}
    try:
        with sync_playwright() as pw:
            options={} if args.chromium is None else {'executable_path':str(args.chromium)}
            browser=pw.chromium.launch(headless=True,**options)
            page=browser.new_page(viewport={'width':1440,'height':1080})
            page.on('pageerror',lambda error:report['errors'].append(str(error)))
            page.goto(args.url.rstrip('/')+'/workcell/')
            info=page.evaluate("async()=>await (await fetch('/api/workcell/info')).json()")
            if info['allow_real']:
                raise PermissionError('Stop: do not run browser regression against a hardware-enabled server')
            expect(page.locator('#connection')).to_contain_text('默认不连接真机')
            assert page.locator('#real').is_disabled()
            page.click('#preview')
            expect(page.locator('#run-status')).to_contain_text('未验证')
            report['checks'].append({'task':'pickplace','preview':'passed'})
            page.screenshot(path=str(args.output/'01_pickplace.png'),full_page=True)
            page.click('[data-task="magnetic"]')
            if args.design:
                design=json.loads(args.design.read_text())
                design['_codex_roundtrip_unknown']={'preserve':True}
                page.locator('#design-file').set_input_files({'name':'audited_builder.json',
                    'mimeType':'application/json','buffer':json.dumps(design).encode()})
                count=sum(not piece.get('locked',False) for piece in design['pieces'])
                expect(page.locator('#piece-count')).to_contain_text(f'{count} / 12')
                with page.expect_download() as pending:
                    page.click('#export-design')
                exported=json.loads(Path(pending.value.path()).read_text())
                assert exported==design,'Builder round-trip changed original/unknown fields'
                report['checks'].append({'task':'magnetic','full_builder_roundtrip':'passed',
                    'pieces':len(design['pieces']),'movable':count,'source':str(args.design)})
            page.click('#preview')
            expect(page.locator('#result')).to_contain_text('builder_y_up_columns_u_n_v')
            report['checks'].append({'task':'magnetic','preview':'passed'})
            page.screenshot(path=str(args.output/'02_magnetic.png'),full_page=True)
            page.click('[data-task="pusht"]');page.fill('#goal-yaw','8');page.click('#simulate')
            expect(page.locator('#run-status')).to_contain_text('模型内达到目标',timeout=30000)
            report['checks'].append({'task':'pusht','cpu_surrogate_loop':'passed'})
            page.screenshot(path=str(args.output/'03_pusht.png'),full_page=True)
            page.fill('#goal-x','0.58');page.fill('#goal-y','0.18');page.fill('#push-steps','500')
            page.click('#simulate');expect(page.locator('#stop')).to_be_enabled()
            page.click('#stop')
            expect(page.locator('#run-status')).to_contain_text('任务已停止',timeout=30000)
            report['checks'].append({'task':'pusht','surrogate_stop':'passed'})
            assert not report['errors'],report['errors']
            browser.close()
    finally:
        (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
