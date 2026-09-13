"""Fixture coverage of original retreat generator and measured native audit."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.pusht.stage_planning import replan_stage
from rm75_app.pusht.observation import Observation

@pytest.mark.parametrize('reject',[False,True])
def test_retreat_uses_measured_scene_and_no_contact_exemption(reject):
    q=np.arange(7)*.01;calls=[]
    obs=Observation('fixture',3,4.,(.38,-.18,0.),'simulation')
    def generate(start,push,scene,binding):
        assert np.array_equal(start,q) and scene is obs and binding=={'original':True}
        calls.append('generate');return np.array([q,q+.001]),np.array([0.,1.])
    def audit(path,contact):
        assert contact is False;calls.append('audit')
        if reject:raise RuntimeError('native collision')
    executor=SimpleNamespace(arm=SimpleNamespace(read_joints=lambda:q),
        backend=SimpleNamespace(_ensure_planner=lambda:SimpleNamespace(
                                    joint_names=tuple(f'joint_{i}' for i in range(1,8))),
                                update_scene=lambda s:calls.append('scene'),
                                set_gripper_collision_state=lambda **kw:None),
        _scene=lambda o:o,_plan_retreat=generate,_audit=audit)
    request=dict(stage='retreat',goal_q=q.tolist(),contact_binding={'original':True},
        push=dict(contact=[.3,-.18],direction=[1,0],length_m=.02,speed_mps=.015))
    if reject:
        with pytest.raises(RuntimeError,match='native collision'):replan_stage(executor,obs,request)
    else:
        result=replan_stage(executor,obs,request)
        assert result['measured_start_q']==q.tolist()
        assert result['source_observation']==obs.as_dict()
        assert result['swm_scene_audit_verified'] is False
    assert calls==['scene','generate','audit']
    assert executor.names==tuple(f'joint_{i}' for i in range(1,8))

def test_wrong_native_joint_order_rejects_before_read_or_plan():
    executor=SimpleNamespace(backend=SimpleNamespace(_ensure_planner=lambda:
        SimpleNamespace(joint_names=('joint_2','joint_1','joint_3','joint_4','joint_5','joint_6','joint_7'))))
    with pytest.raises(ValueError,match='original ordered'):
        replan_stage(executor,None,{'stage':'approach'})
