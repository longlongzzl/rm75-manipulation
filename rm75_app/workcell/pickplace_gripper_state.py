"""Synchronize SIM gripper joints into every cuRobo kinematics owner."""
import copy
import numpy as np


def _sim_gripper_state(planner,demo,args):
    from .pickplace_curobo_only import CuroboOnlyUnsupported
    if planner.attached_object_active:
        raise CuroboOnlyUnsupported('cannot replace gripper geometry with payload attached')
    q=demo.robot.get_qpos()
    if hasattr(q,'detach'):q=q.detach().cpu().numpy()
    q=np.asarray(q,dtype=float).reshape(-1)
    names=list(demo.active_joint_names)
    if len(q)!=len(names) or len(set(names))!=len(names) or not np.isfinite(q).all():
        raise CuroboOnlyUnsupported('invalid simulated gripper joint state')
    desired=dict(planner._pickplace_nominal_gripper_locks)
    if not set(desired)<=set(names):
        raise CuroboOnlyUnsupported('simulated gripper joint identity missing')
    return {name:float(q[names.index(name)]) for name in desired}


def _replace_gripper_locks(planner,raw,current,desired,args):
    from .pickplace_curobo_only import CuroboOnlyUnsupported
    if desired==current:
        return dict(synced=False,before=current,after=current)
    if planner.attached_object_active or planner._disabled_collision_links:
        raise CuroboOnlyUnsupported('cannot replace active attachment or masked robot model: '
            f'attached={planner.attached_object_active}, '
            f'disabled_links={sorted(planner._disabled_collision_links)}')
    updated=copy.deepcopy(raw)
    updated.get('robot_cfg',updated)['kinematics']['lock_joints']=desired
    rebuilt=planner.mods['RobotConfig'].from_dict(updated,tensor_args=planner.tensor_args)
    new_cfg=rebuilt.kinematics.kinematics_config
    targets={id(planner.robot_cfg.kinematics.kinematics_config):planner.robot_cfg.kinematics.kinematics_config}
    owners=[planner.motion_gen,planner.ik_solver,*planner._cuda_graph_batch_ik_solvers.values()]
    for owner in owners:
        rollouts=list(owner.get_all_rollout_instances())
        if getattr(owner,'rollout_fn',None) is not None:rollouts.append(owner.rollout_fn)
        for rollout in rollouts:
            cfg=rollout.kinematics.kinematics_config
            targets[id(cfg)]=cfg
    # cuRobo's supported in-place update keeps CUDA graph tensor addresses.
    # Do not permit a topology/radius change as part of joint synchronization.
    import torch
    attached_id=getattr(new_cfg,'link_name_to_idx_map',{}).get('attached_object')
    attached_mask=(new_cfg.link_sphere_idx_map==attached_id if attached_id is not None
                   else torch.zeros_like(new_cfg.link_sphere_idx_map,dtype=torch.bool))
    for cfg in targets.values():
        if (cfg.link_spheres.shape!=new_cfg.link_spheres.shape or
            not torch.equal(cfg.link_sphere_idx_map,new_cfg.link_sphere_idx_map)):
            raise CuroboOnlyUnsupported('locked-joint update changed sphere topology')
        same=cfg.link_spheres[...,3]==new_cfg.link_spheres[...,3]
        disabled_payload=attached_mask&(cfg.link_spheres[...,3]<0)&(new_cfg.link_spheres[...,3]<0)
        if not bool((same|disabled_payload).all()):
            raise CuroboOnlyUnsupported('locked-joint update changed sphere topology/radii: '
                f'old={cfg.link_spheres[...,3].tolist()} new={new_cfg.link_spheres[...,3].tolist()}')
    for cfg in targets.values():
        # detach uses -100, initial YAML uses another negative sentinel.
        # Keep each owner's disabled payload rows exactly, never reactivate or
        # resize a physical sphere while copying the measured gripper transforms.
        payload=cfg.link_spheres[attached_mask].clone()
        reference=getattr(cfg,'reference_link_spheres',None)
        payload_reference=reference[attached_mask].clone() if reference is not None else None
        cfg.copy_(new_cfg)
        cfg.link_spheres[attached_mask]=payload
        if payload_reference is not None:cfg.reference_link_spheres[attached_mask]=payload_reference
    planner.robot_cfg_dict=updated
    print(f'[curobo gripper] synced {len(targets)} kinematics configs; '
          f'released={bool(getattr(args,"_episode_place_released",False))}; locks={desired}')
    return dict(synced=True,before=current,after=desired)


def sync_gripper_to_demo(planner,demo,args):
    """Always follow the current SIM gripper state (pre-release boundary use).

    The pre-release return check runs while the simulation gripper may already
    be partially opened while the planner model still carries the nominal
    locks; the stale model makes the return START state collide with the
    just-placed piece. Same guards and topology checks as update_from_demo.
    """
    from .pickplace_curobo_only import CuroboOnlyUnsupported
    if getattr(args,'execute_real',False):
        raise CuroboOnlyUnsupported('real gripper geometry requires measured hardware state')
    raw=planner.robot_cfg_dict
    kin=raw.get('robot_cfg',raw)['kinematics']
    current=dict(kin.get('lock_joints') or {})
    if not current:raise CuroboOnlyUnsupported('gripper lock joints missing')
    if not hasattr(planner,'_pickplace_nominal_gripper_locks'):
        planner._pickplace_nominal_gripper_locks=current.copy()
    desired=_sim_gripper_state(planner,demo,args)
    return _replace_gripper_locks(planner,raw,current,desired,args)


def update_from_demo(planner,demo,args):
    from .pickplace_curobo_only import CuroboOnlyUnsupported
    if getattr(args,'execute_real',False):
        raise CuroboOnlyUnsupported('real gripper geometry requires measured hardware state')
    raw=planner.robot_cfg_dict
    kin=raw.get('robot_cfg',raw)['kinematics']
    current=dict(kin.get('lock_joints') or {})
    if not current:raise CuroboOnlyUnsupported('gripper lock joints missing')
    if not hasattr(planner,'_pickplace_nominal_gripper_locks'):
        planner._pickplace_nominal_gripper_locks=current.copy()
    desired=planner._pickplace_nominal_gripper_locks.copy()
    if bool(getattr(args,'_episode_place_released',False)):
        desired=_sim_gripper_state(planner,demo,args)
    return _replace_gripper_locks(planner,raw,current,desired,args)
