#!/usr/bin/env python3
"""Local NORMAL-browser smoke test of the consolidated console.

Does not call the LLM, start a simulation or arm a robot. An optional explicit
--preview-object performs just one preview through the actual isolated worker.
Refuses real-enabled or already-busy servers. Use a separate loopback web process;
its preview/SIM workers remain network-isolated by the existing boundary.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from urllib.parse import urlsplit


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:7861')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--chromium',type=Path)
    parser.add_argument('--preview-object',help='Explicitly submit one PREVIEW only; no SIM or LLM call')
    args=parser.parse_args(argv)
    origin=urlsplit(args.url)
    if origin.scheme!='http' or origin.hostname not in ('127.0.0.1','localhost') or origin.username or origin.password:
        raise ValueError('Use only the loopback workcell URL')
    if args.output.exists():raise FileExistsError('Choose a new evidence directory')
    args.output.mkdir(parents=True)
    report={'scope':'normal browser, real WSGI transport; no LLM/GPU/real task launched',
            'checks':[],'errors':[],'blocked_requests':[],'preview_requested':args.preview_object,
            'normal_navigation':False,'hardware_arming_requested':False}
    from playwright.sync_api import sync_playwright,expect
    base=f'{origin.scheme}://{origin.netloc}'
    try:
        with sync_playwright() as pw:
            options={'headless':True}
            if args.chromium:options['executable_path']=str(args.chromium)
            browser=pw.chromium.launch(**options)
            context=browser.new_context(viewport={'width':1560,'height':1080})
            def route(r):
                req=r.request
                permitted=req.url.startswith(base+'/')
                if req.method=='POST':
                    permitted=permitted and (req.url.endswith('/console/preflight') or
                        req.url.endswith('/console/jobs') and args.preview_object is not None and
                        req.post_data_json.get('spec',{}).get('mode')=='preview')
                if permitted:r.continue_()
                else:report['blocked_requests'].append({'method':req.method,'url':req.url});r.abort()
            context.route('**/*',route)
            page=context.new_page();page.on('pageerror',lambda e:report['errors'].append(str(e)))
            page.goto(base+'/workcell/console/',wait_until='networkidle')
            report['normal_navigation']=True
            boot=page.evaluate("async()=>await (await fetch('/api/workcell/console/bootstrap')).json()")
            if boot['info'].get('allow_real') or boot.get('active_job'):
                raise PermissionError('Refuse a real-enabled or already busy server')
            expect(page.locator('#connection')).to_contain_text('服务器已连接')
            assert page.locator('.task-nav [data-task]').count()==3
            report['checks'].append('three-panel console loads with no page exception')
            if args.preview_object:
                if args.preview_object not in boot['info']['pickplace_objects']:raise ValueError('Unknown configured preview object')
                page.locator(f'#object-grid input[value="{args.preview_object}"]').check()
                page.click('#preview');expect(page.locator('#job-state')).to_contain_text('预览完成',timeout=60000)
                report['checks'].append('explicit single-object preview through actual server/worker')
            page.screenshot(path=str(args.output/'01_pickplace.png'),full_page=True)
            page.click('[data-task=magnetic]')
            if not page.locator('#load-template').is_disabled():
                page.click('#load-template');expect(page.locator('#design-summary')).to_contain_text('原模板')
                report['checks'].append('read-only original template loaded into design viewer')
            page.screenshot(path=str(args.output/'02_jimu.png'),full_page=True)
            page.click('[data-task=pusht]');page.screenshot(path=str(args.output/'03_pusht.png'),full_page=True)
            page.set_viewport_size({'width':390,'height':844});page.evaluate('scrollTo(0,0)')
            assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+2')
            page.screenshot(path=str(args.output/'04_mobile.png'),full_page=True)
            report['checks'].append('mobile layout has no horizontal overflow')
            assert not report['errors'],report['errors']
            report['status']='PASS';browser.close()
    except Exception as exc:
        report['status']='BLOCKED' if 'ERR_BLOCKED_BY_ADMINISTRATOR' in str(exc) else 'FAILED'
        report['error']=str(exc)
        raise
    finally:
        (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0

if __name__=='__main__':raise SystemExit(main())
