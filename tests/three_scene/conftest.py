from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import pytest
from rm75_app.workcell.io import read_json

@pytest.fixture
def profile():
    p=read_json(Path(__file__).resolve().parents[2]/'examples/workcell/machine.example.json')
    for task in ('pickplace','magnetic','pusht'):
        p[task]['python']=sys.executable
    return p

@pytest.fixture
def design():
    return read_json(Path(__file__).resolve().parents[2]/'examples/workcell/jimu_builder_scene_v1.json')
