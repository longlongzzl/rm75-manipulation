"""SIM release-only finger/just-placed-cuboid contact; all other pairs checked."""
from contextlib import contextmanager
import functools
import numpy as np

from .pickplace_curobo_only import CuroboOnlyUnsupported
from .world_only_contact import FINGER_LINKS


class ReleasePathRejected(CuroboOnlyUnsupported):
    """Reject this path, allowing the unchanged native candidate loop to continue."""


def _validate_target_geometry(target):
    dims=np.asarray(getattr(target,'dims',None),dtype=float)
    pose=np.asarray(getattr(target,'pose',None),dtype=float)
    if (getattr(target,'file_path',None) is not None or dims.shape!=(3,) or
        not np.isfinite(dims).all() or not (dims>0).all() or
        pose.shape!=(7,) or not np.isfinite(pose).all() or
        abs(np.linalg.norm(pose[3:])-1)>1e-4):
        raise CuroboOnlyUnsupported('released target geometry is not a qualified cuboid')


def release_target(planner,args,source):
    if getattr(args,'execute_real',False) or not getattr(args,'_episode_place_released',False):
        raise CuroboOnlyUnsupported('target-local release requires released simulation state')
    if not source: raise CuroboOnlyUnsupported('released source identity missing')
    objects=[obj for obj in planner._world.objects if obj.name=='scene_obstacle_'+source]
    if len(objects)!=1 or getattr(objects[0],'dims',None) is None:
        raise CuroboOnlyUnsupported('released target must be one known native cuboid')
    _validate_target_geometry(objects[0])
    return objects[0]


def assert_full_state(planner):
    if (not planner.collision_enabled or not planner.config.self_collision_check or
        planner.attached_object_active or planner._disabled_collision_links or
        set(planner._disabled_world_obstacles)-{'active_target_object'}):
        raise CuroboOnlyUnsupported('unqualified target-local release collision state')


@contextmanager
def target_search_scope(planner,target):
    assert_full_state(planner)
    try:
        changed=planner.set_world_obstacles_enabled([target.name],enabled=False)
        if target.name not in changed:
            raise CuroboOnlyUnsupported('released target collider could not be scoped')
        yield
    finally:
        planner.set_world_obstacles_enabled([target.name],enabled=True)


def audit_release_path(planner,path,target):
    from .pickplace_clearance_audit import sphere_box_contacts
    assert_full_state(planner)
    _validate_target_geometry(target)
    if path is None or len(path)<2:raise CuroboOnlyUnsupported('missing release path')
    q=np.asarray(path,dtype=float)
    if q.ndim!=2 or not np.isfinite(q).all():raise CuroboOnlyUnsupported('invalid release joints')
    dense=[q[0]]
    for a,b in zip(q[:-1],q[1:]):
        count=max(1,int(np.ceil(np.max(abs(b-a))/.01)))
        dense.extend(np.linspace(a,b,count+1)[1:])
    contacts=[];initial_depth=0.
    for index,joints in enumerate(dense):
        spheres=np.asarray(planner._compute_world_link_spheres(joints))
        links=planner._collision_sphere_link_names()
        if spheres.shape!=(len(links),4) or not np.isfinite(spheres).all():
            raise CuroboOnlyUnsupported('invalid native collision spheres')
        pairs=sphere_box_contacts(spheres,links,[target])
        if any(pair['link'] not in FINGER_LINKS for pair in pairs):
            raise ReleasePathRejected(f'non-finger contact with released target at {index}')
        depth=max((pair['overlap_m'] for pair in pairs),default=0.)
        if index==0:initial_depth=depth
        if depth>initial_depth+1e-6:
            raise ReleasePathRejected(f'release contact deeper than initial state at {index}: '
                f'initial_m={initial_depth:.9f}, current_m={depth:.9f}')
        contacts.append(pairs)
    # Native GPU checks ALL self pairs and all links against every other world
    # object. Exact native target cuboid/sphere SDF above covers the missing pair.
    with target_search_scope(planner,target):
        for index,joints in enumerate(dense):
            valid,status=planner.check_start_state(joints)
            if not valid:raise ReleasePathRejected(f'release waypoint {index}: {status}')
    depths=[max((p['overlap_m'] for p in pairs),default=0.) for pairs in contacts]
    max_increase=max([0., *(b-a for a,b in zip(depths[:-1],depths[1:]))])
    return dict(samples=len(path),audited_samples=len(dense),all_valid=True,world_exempt_links=[],
        monotonic_contact_nonincreasing=max_increase<=1e-6,
        max_step_penetration_increase_m=max_increase,
        permitted_contact_target=target.name,permitted_links=sorted(FINGER_LINKS),
        initial_penetration_m=initial_depth,final_penetration_m=max(
            (p['overlap_m'] for p in contacts[-1]),default=0.),
        contact_sample_count=sum(bool(p) for p in contacts))


def install_release_contact(direct):
    records=[];direct._clearance_selection_audits=records
    def acceptable(planner,demo,args,path,*,kind):
        from .pickplace_clearance_audit import validate_clearance_path
        with direct._CUROBO_GPU_LOCK:
            try:
                evidence=validate_clearance_path(planner,path)
            except CuroboOnlyUnsupported:
                target=release_target(planner,args,direct._current_source_object_name(args))
                try:evidence=audit_release_path(planner,path,target)
                except ReleasePathRejected as exc:
                    row={'kind':kind,'accepted':False,'reason':str(exc),'path_points':len(path)}
                    records.append(row);print(f'[clearance selection audit] {row}')
                    return False
            row={'kind':kind,'accepted':True,**evidence}
            records.append(row);print(f'[clearance selection audit] {row}')
            return True

    # Full-path validity supplements the native endpoint check. False uses the
    # SAME already-present fresh-clearance branch; no new path or fallback.
    direct._rm75_clearance_reverse_path_reusable=lambda planner,demo,args,path:acceptable(
        planner,demo,args,path,kind='reverse_reuse')

    original_audit=getattr(direct,'_audit_post_place_clearance_selective_contact_path',None)
    if callable(original_audit):
        @functools.wraps(original_audit)
        def selective(planner,demo,args,path,**kwargs):
            with direct._CUROBO_GPU_LOCK:
                result=original_audit(planner,demo,args,path,**kwargs)
                if (result.get('success') and getattr(args,'_episode_place_released',False)
                        and not acceptable(planner,demo,args,path,kind='released_candidate')):
                    return {**result,'success':False,'status':'REJECTED_RELEASE_PATH_AUDIT'}
                return result
        direct._audit_post_place_clearance_selective_contact_path=selective

    original=direct._plan_constrained_linear_segment
    @functools.wraps(original)
    def segment(planner,demo,args,start_q,pose_start,pose_goal,**kwargs):
        if not str(kwargs.get('label','')).startswith('post_place_clearance'):
            return original(planner,demo,args,start_q,pose_start,pose_goal,**kwargs)
        with direct._CUROBO_GPU_LOCK:
            # First prove the initial invalidity is ONLY qualified finger/target
            # contact. Never use an exception to repair a masked/attached model.
            valid,_=planner.check_start_state(start_q)
            if valid:return original(planner,demo,args,start_q,pose_start,pose_goal,**kwargs)
            target=release_target(planner,args,direct._current_source_object_name(args))
            audit_release_path(planner,[start_q,start_q],target)
            with target_search_scope(planner,target):
                path=original(planner,demo,args,start_q,pose_start,pose_goal,**kwargs)
            if path is None:return None
            try:
                evidence=audit_release_path(planner,path,target)
            except ReleasePathRejected as exc:
                print(f'[release target-local rejected] {kwargs.get("label")}: {exc}')
                return None
            print(f'[release target-local audit] {evidence}')
            return path
    direct._plan_constrained_linear_segment=segment
