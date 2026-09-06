#!/usr/bin/env python3
"""Local browser regression, with normal navigation and the actual HTTP transport.

Run a non-hardware workcell server first. This script refuses a real-enabled
server and never clicks the real or arm endpoint. Playwright/Chromium required.
"""
from pathlib import Path
import argparse
import json
from urllib.parse import urlsplit
from playwright.sync_api import sync_playwright


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:7861')
    parser.add_argument('--output',type=Path,default=Path('validation/local-browser'))
    parser.add_argument('--chromium',type=Path)
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
            page.wait_for_function("document.querySelector('#connection').textContent.includes('默认不连接真机')")
            assert page.locator('#real').is_disabled()
            page.click('#preview')
            page.wait_for_function("document.querySelector('#run-status').textContent.includes('未验证')")
            report['checks'].append({'task':'pickplace','preview':'passed'})
            page.screenshot(path=str(args.output/'01_pickplace.png'),full_page=True)
            page.click('[data-task="magnetic"]');page.click('#preview')
            page.wait_for_function("document.querySelector('#result').textContent.includes('builder_y_up_columns_u_n_v')")
            report['checks'].append({'task':'magnetic','preview':'passed'})
            page.screenshot(path=str(args.output/'02_magnetic.png'),full_page=True)
            page.click('[data-task="pusht"]');page.fill('#goal-yaw','8');page.click('#simulate')
            page.wait_for_function("document.querySelector('#run-status').textContent.includes('模型内达到目标')",timeout=30000)
            report['checks'].append({'task':'pusht','cpu_surrogate_loop':'passed'})
            page.screenshot(path=str(args.output/'03_pusht.png'),full_page=True)
            assert not report['errors'],report['errors']
            browser.close()
    finally:
        (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
