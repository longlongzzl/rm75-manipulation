"""No live endpoints: verify the kernel boundary and its placement in main."""
import ast
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from rm75_app.workcell.offline_boundary import isolate_task_worker
from tools import run_network_isolated


@pytest.mark.parametrize('mode',['sim','preview'])
def test_offline_filter_required_even_if_profile_requests_skip(monkeypatch,mode):
    calls=[];events=[]
    monkeypatch.setattr(run_network_isolated,'block_network',lambda:calls.append('installed'))
    assert isolate_task_worker({'mode':mode,'skip_isolation':True},
           SimpleNamespace(emit=lambda *a,**kw:events.append((a,kw))))
    assert calls==['installed'] and events[0][1]['non_unix_network_denied']
    assert not events[0][1]['serial_usb_isolated']


def test_filter_failure_does_not_dispatch_or_emit_success(monkeypatch):
    def fail():raise RuntimeError('filter install failure')
    monkeypatch.setattr(run_network_isolated,'block_network',fail)
    with pytest.raises(RuntimeError,match='filter install failure'):
        isolate_task_worker({'mode':'sim'},SimpleNamespace(emit=lambda *a,**kw:pytest.fail('false success')))


def test_real_remains_separately_authorized_not_blocked_by_this_helper(monkeypatch):
    monkeypatch.setattr(run_network_isolated,'block_network',lambda:pytest.fail('real gate belongs elsewhere'))
    assert isolate_task_worker({'mode':'real'},SimpleNamespace()) is False


@pytest.mark.parametrize('mode',[None,'SIM','unknown'])
def test_unknown_mode_rejected_before_native_import(mode):
    with pytest.raises(ValueError):isolate_task_worker({'mode':mode},SimpleNamespace())


def test_kernel_denial_in_subprocess_no_addresses_or_connections():
    root=Path(__file__).resolve().parents[1]
    code='''
import errno,socket,json,subprocess,sys
from types import SimpleNamespace
from rm75_app.workcell.offline_boundary import isolate_task_worker
rows=[]
isolate_task_worker({'mode':'sim'},SimpleNamespace(emit=lambda *a,**kw:rows.append(kw)))
for family in (socket.AF_INET,socket.AF_INET6):
    for kind in (socket.SOCK_STREAM,socket.SOCK_DGRAM):
        try:s=socket.socket(family,kind)
        except OSError as e:assert e.errno==errno.EPERM
        else:s.close();raise AssertionError('network socket allowed')
a,b=socket.socketpair();a.close();b.close()
print(json.dumps({'non_unix_denied':True,'unix_ipc_allowed':True,'events':rows}))
'''
    result=subprocess.run([sys.executable,'-c',code],cwd=root,capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr
    report=json.loads(result.stdout)
    assert report['non_unix_denied'] and report['unix_ipc_allowed']


def test_actual_worker_calls_filter_before_engine_dispatch():
    path=Path(__file__).resolve().parents[1]/'rm75_app/workcell/worker.py'
    tree=ast.parse(path.read_text());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    calls=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
    boundary=next(n for n in calls if n.func.id=='isolate_task_worker')
    for name in ('ResourceLease','preview','run_pusht','run_working'):
        assert boundary.lineno<next(n.lineno for n in calls if n.func.id==name)
    assert "if not args.real_authorized" in path.read_text()
