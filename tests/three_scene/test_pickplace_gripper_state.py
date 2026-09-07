import copy
from types import SimpleNamespace as NS
import numpy as np
import pytest
import torch
from rm75_app.workcell.pickplace_gripper_state import update_from_demo
from rm75_app.workcell.pickplace_curobo_only import CuroboOnlyUnsupported


def fixture():
    class Config:
        def __init__(self,value=.6,radius=.02):
            self.link_spheres=torch.tensor([[value,0,0,radius]])
            self.link_sphere_idx_map=torch.tensor([0])
            self.copies=0
        def copy_(self,new):
            self.link_spheres.copy_(new.link_spheres);self.copies+=1
    configs=[Config() for _ in range(4)]
    def owner(cfg):
        rollout=NS(kinematics=NS(kinematics_config=cfg))
        return NS(rollout_fn=rollout,get_all_rollout_instances=lambda:[rollout])
    def build(raw,**kwargs):
        value=raw['robot_cfg']['kinematics']['lock_joints']['finger']
        return NS(kinematics=NS(kinematics_config=Config(value)))
    planner=NS(robot_cfg_dict={'robot_cfg':{'kinematics':{'lock_joints':{'finger':.6}}}},
        robot_cfg=NS(kinematics=NS(kinematics_config=configs[0])),
        motion_gen=owner(configs[1]),ik_solver=owner(configs[2]),
        _cuda_graph_batch_ik_solvers={'batch':owner(configs[3])},
        mods={'RobotConfig':NS(from_dict=build)},tensor_args=None,
        attached_object_active=False,_disabled_collision_links=set())
    demo=NS(active_joint_names=['arm','finger'],robot=NS(get_qpos=lambda:np.array([[1.,.012]])))
    args=NS(execute_real=False,_episode_place_released=True)
    return planner,demo,args,configs


def test_all_kinematics_and_cached_ik_update_in_place_then_restore_next_cycle():
    p,d,a,configs=fixture();pointers=[c.link_spheres.data_ptr() for c in configs]
    update_from_demo(p,d,a)
    assert all(c.copies==1 and c.link_spheres[0,0]==pytest.approx(.012) for c in configs)
    assert pointers==[c.link_spheres.data_ptr() for c in configs]
    assert p.robot_cfg_dict['robot_cfg']['kinematics']['lock_joints']=={'finger':.012}
    a._episode_place_released=False
    update_from_demo(p,d,a)
    assert all(c.copies==2 and c.link_spheres[0,0]==pytest.approx(.6) for c in configs)


@pytest.mark.parametrize('failure',['real','attached','masked','missing_joint','nan','radius'])
def test_unqualified_state_or_geometry_change_fails_before_any_copy(failure):
    p,d,a,configs=fixture()
    if failure=='real':a.execute_real=True
    if failure=='attached':p.attached_object_active=True
    if failure=='masked':p._disabled_collision_links.add('finger')
    if failure=='missing_joint':d.active_joint_names=['arm','wrong']
    if failure=='nan':d.robot.get_qpos=lambda:np.array([[1.,np.nan]])
    if failure=='radius':configs[-1].link_spheres[0,3]=.03
    original=copy.deepcopy(p.robot_cfg_dict)
    with pytest.raises(CuroboOnlyUnsupported):update_from_demo(p,d,a)
    assert all(c.copies==0 for c in configs)
    assert p.robot_cfg_dict==original


def test_disabled_payload_sentinel_is_preserved_not_reactivated_or_resized():
    p,d,a,configs=fixture()
    for cfg in configs:
        cfg.link_spheres=torch.cat([cfg.link_spheres,torch.tensor([[0.,0,0,-100.]])])
        cfg.link_sphere_idx_map=torch.tensor([0,1])
        cfg.link_name_to_idx_map={'finger':0,'attached_object':1}
    original_build=p.mods['RobotConfig'].from_dict
    def build(raw,**kwargs):
        result=original_build(raw,**kwargs);cfg=result.kinematics.kinematics_config
        cfg.link_spheres=torch.cat([cfg.link_spheres,torch.tensor([[0.,0,0,-9.9975]])])
        cfg.link_sphere_idx_map=torch.tensor([0,1]);cfg.link_name_to_idx_map={'finger':0,'attached_object':1}
        return result
    p.mods['RobotConfig'].from_dict=build
    update_from_demo(p,d,a)
    assert all(c.link_spheres[1,3]==-100 and c.link_spheres[0,3]==pytest.approx(.02) for c in configs)
