import json
from pathlib import Path
import subprocess
import pytest


@pytest.mark.parametrize('height',[.08,.3,1.0])
def test_full_height_builder_projection_fits_without_changing_design(height):
    root=Path(__file__).resolve().parents[2]
    prefix=(root/'rm75_app/web/static/workcell/app.js').read_text().split("for(const button of document.querySelectorAll")[0]
    pieces=[dict(type='square',center=[0,0,0],u=[1,0,0],v=[0,0,1]),
            dict(type='triangle',center=[.04,height,0],u=[1,0,0],v=[0,1,0])]
    code=prefix+'\nconst input='+json.dumps(pieces)+''';
const before=JSON.stringify(input),project=magneticProjection(input);
process.stdout.write(JSON.stringify({points:input.flatMap(magneticVertices).map(project),unchanged:before===JSON.stringify(input)}));'''
    result=subprocess.run(['node','-e',code],check=True,capture_output=True,text=True)
    data=json.loads(result.stdout)
    assert data['unchanged'] and len(data['points'])==7
    assert all(29.99<=x<=770.01 and 47.99<=y<=416.01 for x,y in data['points'])
