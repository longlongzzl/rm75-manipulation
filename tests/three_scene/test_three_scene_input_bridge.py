import os
import subprocess
import sys
import time
from pathlib import Path
from rm75_app.workcell.io import read_json


def test_actual_python_child_prompt_waits_for_explicit_stdin(tmp_path):
    root=Path(__file__).resolve().parents[2]
    env=os.environ.copy()
    env['PYTHONPATH']=str(root)
    env['RM75_WORKCELL_INPUT_DIR']=str(tmp_path)
    provider=tmp_path/'provider.py'
    provider.write_text("value=input('child confirmation: '); print('answer='+repr(value))")
    from rm75_app.workcell.input_bridge import install_subprocess_bridge
    original=install_subprocess_bridge(tmp_path)
    try:
        child=subprocess.Popen([sys.executable,str(provider)],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    finally:
        subprocess.Popen=original
    try:
        path=tmp_path/'pending_input.json';deadline=time.monotonic()+3
        while not path.exists() and time.monotonic()<deadline:time.sleep(.01)
        record=read_json(path)
        assert record['pid']==child.pid and record['prompt']=='child confirmation: '
        assert child.poll() is None
        output,error=child.communicate(b'r\n',timeout=3)
        assert child.returncode==0 and b"answer='r'" in output and error==b''
        assert not path.exists()
    finally:
        if child.poll() is None:child.kill();child.wait()
