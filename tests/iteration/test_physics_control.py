from types import SimpleNamespace
import sys
import numpy as np
import pytest
from rm75_app.pusht.physics_controls import relocate_idle_target
from rm75_app.pusht.model import Config


def fixture(monkeypatch):
    writes=[]
    monkeypatch.setitem(sys.modules,'sapien',SimpleNamespace(Pose=lambda p,q:(p,q)))
    target=SimpleNamespace(set_pose=lambda p:writes.append(('pose',p)),
        set_linear_velocity=lambda p:writes.append(('linear',p)),set_angular_velocity=lambda p:writes.append(('angular',p)))
    transform=np.eye(4);transform[:3,3]=[.1,0,.3]
    session=SimpleNamespace(kind='full_arm_physics',spec={'mode':'sim'},report={'hardware_connected':False},config=Config(),
        base=SimpleNamespace(active_program=SimpleNamespace(duration=1),program_started=0,physics_time=2,target=target,read_q=lambda:np.zeros(7)),
        physical_state=lambda:(np.array([.35,0,0]),{}),fk=lambda q:transform,
        planner_geometry={'spheres':[[0,0,0,.01]]},motion={'object_centroid_z_m':.02,'object_height_m':.04},
        events=SimpleNamespace(emit=lambda *args,**kw:None))
    return session,writes


def test_explicit_physics_intervention_logged_not_learned(monkeypatch):
    s,writes=fixture(monkeypatch);relocate_idle_target(s,[.36,0,.1],1)
    assert [x[0] for x in writes]==['pose','linear','angular']
    row=s.report['external_interventions'][0]
    assert row['source']=='explicit_external_sim_intervention' and not row['counted_as_push'] and not row['response_fit_allowed']
    assert not s.report['target_driven_by_physics_only']


@pytest.mark.parametrize('change', ['real','active','outside','hardware'])
def test_invalid_intervention_never_changes_target(monkeypatch,change):
    s,writes=fixture(monkeypatch)
    if change=='real':s.spec['mode']='real'
    if change=='active':s.base.physics_time=.5
    if change=='hardware':s.report['hardware_connected']=True
    with pytest.raises((ValueError,RuntimeError,PermissionError)):
        relocate_idle_target(s,[5,0,0] if change=='outside' else [.36,0,0],1)
    assert not writes


def test_tool_overlap_rejects_before_teleport(monkeypatch):
    s,writes=fixture(monkeypatch);s.fk=lambda q:np.array([[1,0,0,.36],[0,1,0,0],[0,0,1,.02],[0,0,0,1]])
    with pytest.raises(ValueError,match='tool safety envelope'):relocate_idle_target(s,[.36,0,0],1)
    assert not writes
