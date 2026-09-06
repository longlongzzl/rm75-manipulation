import shutil
import subprocess
from pathlib import Path

import pytest


def test_workcell_javascript_is_not_truncated():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node required for JavaScript syntax validation')
    source = Path(__file__).resolve().parents[2] / 'rm75_app/web/static/workcell/app.js'
    subprocess.run([node, '--check', str(source)], check=True, capture_output=True)
