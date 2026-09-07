"""Read-only clearance evidence; no planner seeds, geometry, or filters changed."""
from contextlib import contextmanager
import functools
import inspect
import numpy as np
from .contact_audit import scene_evidence,_plain
from .transforms import quaternion_matrix


def install_execution_guards(base,lock,records,*,released_source=None):
    from .pickplace_curobo_only import CuroboOnlyUnsupported
    for name in ('execute_pose_path_stage','execute_joint_path_stage'):
        original=getattr(base,name)
        if getattr(original,'_clearance_execution_guard',False):continue
        def wrap(original):
            signature=inspect.signature(original)
            @functools.wraps(original)
            def checked(*pos,**kwargs):
                bound=signature.bind(*pos,**kwargs).arguments
                label=bound['label']
                if not str(label).startswith('post_place_clearance'):
                    return original(*pos,**kwargs)
                with lock:
                    native=getattr(bound['demo'].planner,'native',None)
                    if native is None:raise CuroboOnlyUnsupported('clearance planner not bound')
                    try:
                        evidence=validate_clearance_path(native,bound['q_path'])
                    except CuroboOnlyUnsupported:
                        if released_source is None:raise
                        from .pickplace_release_contact import release_target,audit_release_path
                        target=release_target(native,bound['args'],released_source(bound['args']))
                        evidence=audit_release_path(native,bound['q_path'],target)
                    records.append({'stage':label,**evidence})
                    print(f'[curobo clearance audit] {label}: {evidence}')
                    return original(*pos,**kwargs)
            checked._clearance_execution_guard=True
            return checked
        setattr(base,name,wrap(original))


def validate_clearance_path(planner,path):
    from .pickplace_curobo_only import CuroboOnlyUnsupported
    if (not planner.collision_enabled or not planner.config.self_collision_check or
        planner._disabled_collision_links or planner.attached_object_active or
        set(planner._disabled_world_obstacles)-{'active_target_object'}):
        raise CuroboOnlyUnsupported('unqualified post-release collision state: '
            f'world={planner.collision_enabled}, self={planner.config.self_collision_check}, '
            f'attached={planner.attached_object_active}, '
            f'disabled_links={sorted(planner._disabled_collision_links)}, '
            f'disabled_world={sorted(planner._disabled_world_obstacles)}')
    if path is None or len(path)<2:raise CuroboOnlyUnsupported('missing clearance path')
    for index,q in enumerate(path):
        valid,status=planner.check_start_state(q)
        if not valid:raise CuroboOnlyUnsupported(f'clearance waypoint {index}: {status}')
    return {'samples':len(path),'all_valid':True,'world_exempt_links':[]}


def sphere_box_contacts(spheres, links, objects, disabled=()):
    contacts=[]
    for obj in objects:
        dims=getattr(obj,'dims',None)
        if dims is None:continue
        pose=np.asarray(obj.pose,dtype=float)
        local=(np.asarray(spheres)[:,:3]-pose[:3])@quaternion_matrix(pose[3:])
        delta=np.abs(local)-np.asarray(dims)/2
        distance=np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(delta.max(axis=1),0)-spheres[:,3]
        for i in np.flatnonzero((distance<0)&(spheres[:,3]>0)):
            contacts.append({'link':links[i],'obstacle':obj.name,'sphere_index':int(i),
                             'overlap_m':float(-distance[i]),'enabled':obj.name not in disabled})
    return contacts


def install(direct, emit):
    original_segment=direct._plan_constrained_linear_segment
    @functools.wraps(original_segment)
    def segment(planner,demo,args,start_q,pose_start,pose_goal,**kwargs):
        label=kwargs.get('label','')
        if 'post_place_clearance' not in label:
            return original_segment(planner,demo,args,start_q,pose_start,pose_goal,**kwargs)
        with direct._CUROBO_GPU_LOCK:
            spheres=planner._compute_world_link_spheres(start_q)
            valid,status=planner.check_start_state(start_q)
            emit({'event':'clearance_start','label':label,'q':_plain(start_q),'valid':valid,'status':status,
                'spheres':spheres.tolist(),'sphere_links':planner._collision_sphere_link_names(),
                'sim_joint_names':demo.active_joint_names,'sim_qpos':_plain(demo.robot.get_qpos()),
                'planner_lock_joints':_plain(planner.robot_cfg_dict.get('robot_cfg',planner.robot_cfg_dict)['kinematics'].get('lock_joints')),
                'box_contacts':sphere_box_contacts(spheres,planner._collision_sphere_link_names(),
                    planner._world.objects,planner._disabled_world_obstacles),**scene_evidence(planner)})
            result=original_segment(planner,demo,args,start_q,pose_start,pose_goal,**kwargs)
            emit({'event':'clearance_segment_result','label':label,'path_points':len(result or [])})
            return result
    direct._plan_constrained_linear_segment=segment
    original_profile=direct._profile_stage
    @contextmanager
    def profile(args,name,*pos,**kwargs):
        with original_profile(args,name,*pos,**kwargs) as data:
            try:yield data
            finally:
                if name in ('post_place_clearance','return_to_start','place_open_gripper'):
                    emit({'event':'clearance_profile','stage':name,'data':_plain(data)})
    direct._profile_stage=profile
    from curobo_rm75_planner import RM75CuRoboPlanner
    original_diag=RM75CuRoboPlanner.diagnose_start_state_world_collision
    @functools.wraps(original_diag)
    def diagnose(planner,*args,**kwargs):
        with direct._CUROBO_GPU_LOCK:
            before=sorted(planner._disabled_world_obstacles)
            result=original_diag(planner,*args,**kwargs)
            emit({'event':'native_diagnostic_state','before_disabled':before,
                  'after_disabled':sorted(planner._disabled_world_obstacles),'result':_plain(result)})
            return result
    RM75CuRoboPlanner.diagnose_start_state_world_collision=diagnose
