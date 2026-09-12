"""SDK contract fixtures for complete fitted-slot propagation; no GPU claims."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.planning.backends.curobo2 import Curobo2Backend


class Tensor(np.ndarray):
    def clone(self): return self.copy()


class Parameters:
    def __init__(self,configs=1,slots=64):
        self.link_spheres=np.zeros((configs,slots,4)).view(Tensor)
        self.link_spheres[:,:,3]=-100.
        self.writes=[]
    def get_link_spheres(self,name,config_idx=0):
        assert name=='attached_object'
        return self.link_spheres[config_idx]
    def update_link_spheres(self,name,values,config_idx=None):
        assert name=='attached_object'
        self.writes.append(config_idx)
        self.link_spheres[config_idx]=values


def setup(configs=1,slots=64):
    source=Parameters(); target=Parameters(configs,slots)
    backend=Curobo2Backend.__new__(Curobo2Backend)
    backend._planner=SimpleNamespace(joint_names=('q',),tool_frames=('tcp',),scene_collision_checker={})
    backend._coarse_ik_solver=SimpleNamespace(kinematics=SimpleNamespace(joint_names=('q',),
        tool_frames=('tcp',),config=SimpleNamespace(kinematics_config=target)),scene_collision_checker={})
    manager=SimpleNamespace(kinematics_params=source)
    backend._attachment_manager=lambda:manager
    backend._ensure_planner=lambda:backend._planner
    return backend,source,target,manager


def test_same_fit_refinement_and_detach_propagate_complete_slots():
    backend,source,target,manager=setup()
    source.link_spheres[0,:4]=[[.01,.02,.03,.004]]*4
    backend._sync_coarse_attachment()
    np.testing.assert_array_equal(target.link_spheres,source.link_spheres)
    source.link_spheres[0,:4,0]+=.003
    backend._sync_coarse_attachment()
    np.testing.assert_array_equal(target.link_spheres,source.link_spheres)
    def detach(**kwargs): source.link_spheres[:,:,3]=-100.
    manager.detach=detach
    backend._attachment_active=True
    backend._set_obstacle_enabled=lambda *args:None
    backend.set_gripper_collision_state=lambda closed:None
    backend.detach_object('pen')
    np.testing.assert_array_equal(target.link_spheres,source.link_spheres)
    assert np.all(target.link_spheres[:,:,3]<0)


@pytest.mark.parametrize('fault',['configs','slots','joints','tool'])
def test_incompatible_consumer_rejected_before_any_write(fault):
    backend,source,target,_=setup(configs=2 if fault=='configs' else 1,slots=32 if fault=='slots' else 64)
    if fault=='joints': backend._coarse_ik_solver.kinematics.joint_names=('wrong',)
    if fault=='tool': backend._coarse_ik_solver.kinematics.tool_frames=('wrong',)
    with pytest.raises(RuntimeError,match='differs'): backend._sync_coarse_attachment()
    assert target.writes==[]


def test_world_enable_updates_both_existing_consumers():
    backend,_,_,_=setup()
    def enable(collision,name,value): collision[name]=value
    backend._set_collision_obstacle_enabled=enable
    backend._set_obstacle_enabled('pen',False)
    assert backend._planner.scene_collision_checker=={'pen':False}
    assert backend._coarse_ik_solver.scene_collision_checker=={'pen':False}
    backend.enable_object_collision('pen')
    assert backend._coarse_ik_solver.scene_collision_checker['pen'] is True


def test_world_pose_updates_both_existing_consumers():
    backend,_,_,_=setup()
    backend._update_collision_obstacle_pose=lambda collision,name,value:collision.update({name:value})
    backend._update_obstacle_pose('pen','measured_pose')
    assert backend._planner.scene_collision_checker['pen']=='measured_pose'
    assert backend._coarse_ik_solver.scene_collision_checker['pen']=='measured_pose'
