"""Software coverage of the native planner's additional rejection gate."""
from types import SimpleNamespace
import numpy as np
import pytest
from tools.pusht_physics_planner import audit_prepared_retreat
from tools.pusht_physics_planner import audit_predicted_retreat
from rm75_app.pusht.model import Config,Push,predict
from rm75_app.pusht.observation import Observation

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

@pytest.mark.parametrize('reject',[False,True])
def test_original_ensemble_and_scene_restoration(reject):
    scenes=[];audited=[]
    config=Config();obs=Observation('fixture',1,2.,(.35,-.18,0.),'simulation')
    push=Push((.31,-.18),(1.,0.),.02,.015)
    def audit(path,**kwargs):
        audited.append(scenes[-1])
        if reject:raise RuntimeError('collision')
    executor=SimpleNamespace(config=config,
        last_contact_binding={'surface_contact_xyz':[.30,-.18,.01]},
        _scene=lambda observation:observation,
        backend=SimpleNamespace(update_scene=scenes.append),_audit=audit,
        events=SimpleNamespace(emit=lambda *a,**kw:None))
    prepared=SimpleNamespace(stages=(('retreat',np.zeros((2,7)),None),))
    if reject:
        with pytest.raises(RuntimeError,match='collision'):
            audit_predicted_retreat(executor,prepared,push,obs)
    else:audit_predicted_retreat(executor,prepared,push,obs)
    assert scenes[-1] is obs
    assert len(audited)==(1 if reject else len(config.friction_scales))
    bound=Push((.30,-.18),push.direction,push.length_m,push.speed_mps)
    for row,scale in zip(audited,config.friction_scales):
        assert row.source=='prediction'
        assert row.pose==pytest.approx(predict(obs.pose,bound,config,scale))
    assert obs.source=='simulation' and obs.pose==(.35,-.18,0.)
