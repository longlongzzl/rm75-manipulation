"""Synthetic schema fixtures, never claimed to be the user's original arc board."""
import copy
from collections import Counter
from pathlib import Path
import pytest
from rm75_app.workcell.io import digest,atomic_json


def make_library():
    boards={}
    for key,count in [('grid_3x3',9),('arc',5)]:
        pieces=[]
        for i in range(count):
            pieces.append(dict(id=f'base_{i}',role=f'base_{i}',type='square',locked=True,
                center=[(i%3-1)*.074,.00325,(i//3-1)*.074],u=[1,0,0],n=[0,1,0],v=[0,0,1]))
        for side in range(4):
            for layer in range(3):
                role=f'wall_{side}_{layer}'
                pieces.append(dict(id=role,role=role,type='triangle' if layer==2 else 'square',locked=False,
                    parentId=f'base_{side}' if layer==0 else f'wall_{side}_{layer-1}',
                    center=[(side-1.5)*.09,.0435+.08*layer,.15],u=[1,0,0],n=[0,0,-1],v=[0,1,0],
                    opaque_extension={'preserve':['custom',side,layer]}))
        payload=dict(schema='jimu_builder_scene_v1',pieces=pieces,unknown_original_field={'kept':True})
        moving=[p for p in pieces if not p['locked']]
        t=dict(id=key+'_00',title='Synthetic schema fixture, NOT old board',design=payload,
               design_digest=digest(payload),locked_digest=digest(pieces[:count]),
               inventory=dict(Counter(p['type'] for p in moving)),native_recipe={'native_args':[]},support_dependencies={})
        boards[key]={'title':key,'templates':[t]}
    return dict(schema='rm75_jimu_design_library_v1',boards=boards)


@pytest.fixture
def library():return make_library()

@pytest.fixture
def profile(tmp_path,library):
    path=tmp_path/'library.json';atomic_json(path,library)
    return dict(schema='rm75_workcell_machine_v1',hardware={'hardware_reviewed':False},
        magnetic={'design_library':str(path),'llm':{'enabled':False}},
        pickplace={'object_names':['shuazi','bi','lvmukuai','carriot','tennis','gluestick','hongshupian'],
                   'fixed_scene_format':'native_world','simulation_contact_policy':'transport_world_checked_compatibility'},
        pusht={'model':{},'physics':{'enabled_backends':['tool_only_physics','full_arm_physics'],
          'geometry_variants':{'wide':{'bar_width_m':.12}}}})

@pytest.fixture
def proposal():
    return dict(board_id='grid_3x3',template_id='grid_3x3_00',title='two-level facade',
                selected_roles=['wall_0_0','wall_0_1','wall_0_2'],explanation='All explicit parents are included')
