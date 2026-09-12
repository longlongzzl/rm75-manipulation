"""Readback contract fixtures; no GPU qualification is claimed here."""
import numpy as np
import pytest
from rm75_app.swm.native_audit import CuroboNativeStageAuditor
from rm75_app.swm.scene import SceneInvalid
from .test_native_stage_audit import Model, case


def setup(fault=None):
    snapshot,_=case(); model=Model(snapshot)
    flags={'a':False,'b':True}
    if fault=='held_enabled': flags['a']=True
    if fault=='other_disabled': flags['b']=False
    model._obstacle_enabled=lambda oid:flags[oid]
    model._attachment_fit_report={'object_name':'b' if fault=='identity' else 'a'}
    owners=[dict(owner=name,active_counts=[0 if fault=='empty' else 4]) for name in ('trajectory','planner','ik')]
    if fault=='missing_owner': owners.pop()
    model.read_attachment_collision_state=lambda:dict(source='native_GPU_attachment_link_spheres',
        consumers_consistent=fault!='disagree',owners=owners)
    if fault=='missing_readback': model.read_attachment_collision_state=None
    calls=[]
    def diagnostics(planner,states,**kwargs):
        calls.append(kwargs['ignored_world_objects'])
        return [dict(world_object='b',robot_link='attached_object',penetration_m=.01)]
    model._collision_diagnostics_for_states=diagnostics
    auditor=CuroboNativeStageAuditor(model)
    auditor._collision_holding='a'; auditor._native_audit=True
    return auditor,model,calls


def test_only_verified_duplicate_is_excluded_other_collision_retained():
    auditor,model,calls=setup()
    contacts=auditor._contacts(np.zeros((1,7)),model.names,set())
    assert calls==[{'a'}]
    assert contacts[0]['world_object']=='b'
    assert auditor.last_duplicate_exclusion['active_spheres']==4


@pytest.mark.parametrize('fault',['held_enabled','other_disabled','identity','empty','missing_owner','disagree','missing_readback'])
def test_unverified_duplicate_never_skipped(fault):
    auditor,model,calls=setup(fault)
    with pytest.raises((SceneInvalid,NotImplementedError)):
        auditor._contacts(np.zeros((1,7)),model.names,set())
    assert calls==[]
