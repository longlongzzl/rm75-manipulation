from types import SimpleNamespace as NS
import pytest
from rm75_app.swm.tool_contact_evidence import ToolContactEvidence
from rm75_app.swm.scene import SceneInvalid


def contact(a,b,impulse=(0,0,1),separation=-.001):
    return NS(bodies=[NS(entity=a),NS(entity=b)],points=[NS(impulse=impulse,separation=separation)])


def test_contact_aggregation_preserves_link_stage_and_impulse():
    tool,target=object(),object();e=ToolContactEvidence({tool:'finger'},{target:'T'},'T')
    e.observe([contact(tool,target)],time_s=.1,stage='push')
    e.observe([contact(tool,target,(0,0,2))],time_s=.2,stage='push')
    row=e.report()['pairs'][0]
    assert row['contacts']==2 and row['positive_impulse_contacts']==2
    assert row['maximum_impulse_Ns']==2 and row['kind']=='target' and row['tool_link']=='finger'


def test_no_contact_is_not_path_clearance():
    e=ToolContactEvidence({}, {},'T');e.observe([],time_s=.1,stage='approach')
    assert not e.report()['absence_of_contact_proves_clearance']
    assert not e.report()['path_safety_qualified']


def test_contact_unknown_body_and_evidence_overflow_reject():
    tool,target=object(),object();e=ToolContactEvidence({tool:'finger'},{target:'T'},'T',maximum_pairs=1)
    with pytest.raises(SceneInvalid,match='unregistered'):e.observe([contact(tool,object())],time_s=.1,stage='push')
    e.observe([contact(tool,target)],time_s=.2,stage='push')
    with pytest.raises(SceneInvalid,match='budget'):e.observe([contact(tool,target)],time_s=.3,stage='retreat')
