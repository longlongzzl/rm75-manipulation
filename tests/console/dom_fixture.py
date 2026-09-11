#!/usr/bin/env python3
"""Offline DOM component checks + in-process WSGI fixture, ZERO network.

This process never listens on a network socket. It never imports real IterationAPI,
WorkcellService, worker, SDK or solver. Author-created HTML/JS is rendered in an empty document with a fake fetch bridge.
This is NOT normal navigation, HTTP/CSP, native/LLM/physics qualification.
"""
from __future__ import annotations
import json
import io
import re
from pathlib import Path
import sys
import tempfile
import threading
import time
from socketserver import ThreadingMixIn
from urllib.parse import urlsplit
from wsgiref.simple_server import make_server,WSGIServer,WSGIRequestHandler

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(Path(__file__).parent))
from fixtures import FakeService,install_substitutes
from rm75_app.workcell.io import read_json,atomic_json

class Patch:
    def setitem(self,mapping,key,value):mapping[key]=value


def main():
    from playwright.sync_api import sync_playwright,expect
    output=Path(sys.argv[1] if len(sys.argv)>1 else '/mnt/data/rm75_console_dom')
    output.mkdir(parents=True,exist_ok=True)
    report={'runtime':'UI_FIXTURE_ONLY','network_model_calls':0,'robot_imported':False,'hardware_connected':False,
            'native_worker_run':False,'physics_run':False,'transport':'in_process_fixture_bridge_no_network',
            'storage':'component_map_substitute','normal_browser_navigation':'BLOCKED_BY_ADMINISTRATOR_NOT_BYPASSED','checks':[],'page_errors':[]}
    install_substitutes(Patch())
    from rm75_app.workcell.server import WorkcellWSGI
    root=Path(tempfile.mkdtemp(prefix='console_ui_fixture_'))
    service=FakeService(root);app=WorkcellWSGI(service)
    requests=[]
    def fetch(path,method,headers,body):
        requests.append((method,path))
        raw=(body or '').encode();head=[]
        env={'PATH_INFO':path,'REQUEST_METHOD':method,'HTTP_HOST':'127.0.0.1:7861',
             'CONTENT_LENGTH':str(len(raw)),'wsgi.input':io.BytesIO(raw),
             'HTTP_X_WORKCELL_TOKEN':headers.get('X-Workcell-Token','')}
        data=b''.join(app(env,lambda status,items:head.append((status,items))))
        return {'status':int(head[0][0][:3]),'text':data.decode()}
    static=ROOT/'rm75_app/web/static/workcell/console'
    html=(static/'index.html').read_text()
    html=re.sub(r'<link rel="stylesheet"[^>]*>','',html)
    html=re.sub(r'<script type="module"[^>]*></script>','',html)
    bundle='\n'.join(re.sub(r'^export ', '',(static/f).read_text(),flags=re.M) for f in ('state.js','views.js'))
    app_js=re.sub(r'^import[\s\S]*?;\n','',(static/'app.js').read_text(),flags=re.M)
    bundle+='\n'+app_js
    def mount(context,saved=None):
        page=context.new_page();page.on('pageerror',lambda e:report['page_errors'].append(str(e)))
        page.expose_function('fixture_fetch',fetch)
        page.set_content(html)
        page.add_style_tag(content=(static/'console.css').read_text())
        page.add_script_tag(content='window.__fixtureStorage='+json.dumps(saved or {})+';'+"""
          Object.defineProperty(window,'localStorage',{value:{
            getItem:k=>window.__fixtureStorage[k]||null,
            setItem:(k,v)=>window.__fixtureStorage[k]=v,
            removeItem:k=>delete window.__fixtureStorage[k]}});
          window.fetch=async (path,opts={})=>{
            if(window.__networkDown)throw new TypeError('Offline API fixture');
            const r=await fixture_fetch(path,opts.method||'GET',opts.headers||{},opts.body||'');
            return new Response(r.text,{status:r.status,headers:{'Content-Type':'application/json'}});
          };
        """)
        page.add_script_tag(content='(()=>{'+bundle+'})()')
        return page
    def check(name):report['checks'].append({'name':name,'status':'PASS'})
    try:
        with sync_playwright() as pw:
            browser=pw.chromium.launch(headless=True,executable_path='/usr/bin/chromium',args=['--no-sandbox','--disable-dev-shm-usage'])
            context=browser.new_context(viewport={'width':1560,'height':1080},device_scale_factor=1)
            context.route('**/*',lambda r:r.abort())
            page=mount(context)
            expect(page.locator('#connection')).to_contain_text('服务器已连接')
            assert page.locator('.task-nav [data-task]').count()==3
            assert page.locator('#simulate').is_disabled()
            check('loads one consolidated console, no automatic task')
            page.click('#select-all');page.click('#preview')
            expect(page.locator('#job-state')).to_contain_text('预览完成',timeout=10000)
            assert len(service.submissions)==1
            page.screenshot(path=str(output/'01_pickplace.png'),full_page=True)
            saved=page.evaluate('window.__fixtureStorage');page.close();page=mount(context,saved)
            expect(page.locator('#restore-note')).to_contain_text('恢复')
            assert page.locator('#object-grid input:checked').count()==7 and len(service.submissions)==1
            check('preview + component remount restores draft and never repeats submission')
            page.click('[data-task=magnetic]');page.click('#load-template')
            expect(page.locator('#design-summary')).to_contain_text('原模板')
            assert page.locator('#piece-list span').count()==3
            check('original-template design/proof loads into canvas')
            app.iteration.delay=4
            page.click('#generate')
            expect(page.locator('#generation-status')).to_contain_text('生成中',timeout=10000)
            assert len(app.iteration.calls)==1
            ident=next(iter(app.iteration._jobs));meta=root/'runtime_data/workcell/design_generations'/ident/'console_meta.json'
            value=read_json(meta);value['created_at']-=200;atomic_json(meta,value)
            saved=page.evaluate('window.__fixtureStorage');page.close();page=mount(context,saved)
            expect(page.locator('#generation-status')).to_contain_text('生成中',timeout=2000)
            assert len(app.iteration.calls)==1
            expect(page.locator('#design-summary')).to_contain_text('LLM 生成',timeout=10000)
            assert len(app.iteration.calls)==1
            check('slow generation >135s logical elapsed + component remount resumes without another call')
            page.screenshot(path=str(output/'02_jimu.png'),full_page=True)
            page.click('#simulate');expect(page.locator('#confirm-dialog')).to_be_visible()
            before=len(service.submissions);page.locator('#confirm-dialog [value=cancel]').click();assert len(service.submissions)==before
            page.click('#simulate');page.locator('#confirm-dialog [value=run]').click()
            expect(page.locator('#stop')).to_be_enabled()
            job=service.active;assert job and service.submissions[-1]['parameters']['generation_proof']
            service.complete(job);expect(page.locator('#job-state')).to_contain_text('未验证',timeout=10000)
            check('cancel confirmation does not run; explicit SIM submits identical proof to runtime substitute')
            page.click('[data-task=pusht]')
            page.click('#simulate');page.locator('#confirm-dialog [value=run]').click()
            expect(page.locator('#stop')).to_be_enabled();page.locator('#intervention-panel summary').click()
            expect(page.locator('#pause')).to_be_enabled();page.click('#pause')
            expect(page.locator('#job-phase')).to_contain_text('已在动作边界暂停',timeout=10000)
            expect(page.locator('#relocate')).to_be_enabled();page.click('#relocate')
            expect(page.locator('#control-status')).to_contain_text('已确认',timeout=10000)
            page.click('#resume');expect(page.locator('#job-phase')).to_contain_text('重新观察',timeout=10000)
            assert app.iteration.controls==['pause','relocate','resume']
            check('pause acknowledgement gates relocate and resume through existing session routes')
            page.screenshot(path=str(output/'03_pusht.png'),full_page=True)
            page.click('#stop');expect(page.locator('#job-state')).to_contain_text('已停止',timeout=10000)
            assert len(service.stops)==1
            check('single shared Stop waits for terminal job result')
            with page.expect_download() as download:
                page.click('#export-report')
            path=output/'diagnostic_fixture_report.json';download.value.save_as(path)
            data=json.loads(path.read_text());assert 'profile' not in data and 'hardware' not in data
            check('bounded diagnostic report export excludes full machine profile')
            page.evaluate('window.__networkDown=true');page.click('#refresh')
            expect(page.locator('#connection')).to_contain_text('连接中断',timeout=15000)
            previous=len(service.submissions);page.evaluate('window.__networkDown=false');page.click('#refresh')
            expect(page.locator('#connection')).to_contain_text('服务器已连接',timeout=15000)
            assert len(service.submissions)==previous
            check('offline/reconnect preserves task identities without resubmission')
            page.set_viewport_size({'width':390,'height':844});page.evaluate('scrollTo(0,0)')
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth+2')
            page.screenshot(path=str(output/'04_mobile.png'),full_page=True)
            check('390px layout has no horizontal overflow')
            assert not report['page_errors'],report['page_errors']
            assert not any('/arm' in url for _,url in requests)
            check('no page errors, no arming requests')
            report.update(status='PASS',inprocess_requests=len(requests),task_submissions=len(service.submissions),model_substitute_calls=len(app.iteration.calls))
            browser.close()
    except Exception as exc:
        report.update(status='FAILED',error=str(exc))
        raise
    finally:
        atomic_json(output/'report.json',report)
    return 0

if __name__=='__main__':raise SystemExit(main())
