"""Software coverage of the native planner's additional rejection gate."""
from types import SimpleNamespace
import numpy as np
import pytest
from tools.pusht_physics_planner import audit_prepared_retreat

def test_all_timed_retreat_samples_checked_without_contact_permission():
    calls=[];events=[];path=np.zeros((11,7))
    executor=SimpleNamespace(_audit=lambda p,**kw:calls.append((p,kw)),
        events=SimpleNamespace(emit=lambda *a,**kw:events.append(kw)))
    prepared=SimpleNamespace(stages=(('push',path,None),('retreat',path,None)))
    audit_prepared_retreat(executor,prepared)
    assert len(calls)==1 and calls[0][0] is path
    assert calls[0][1]=={'contact':False}
    assert events[0]['post_push_scene_verified'] is False

def test_native_collision_failure_propagates():
    def reject(*args,**kwargs):raise RuntimeError('native collision')
    events=[]
    executor=SimpleNamespace(_audit=reject,
        events=SimpleNamespace(emit=lambda *a,**kw:events.append((a,kw))))
    with pytest.raises(RuntimeError,match='native collision'):
        audit_prepared_retreat(executor,SimpleNamespace(stages=(('retreat',np.zeros((2,7)),None),)))
    assert events[0][0]==('physics_retreat_native_rejected',)
    assert events[0][1]['samples']==2
    assert events[0][1]['error']=='RuntimeError: native collision'

def test_absent_retreat_not_certified():
    with pytest.raises(ValueError,match='no retreat'):
        audit_prepared_retreat(None,SimpleNamespace(stages=()))
