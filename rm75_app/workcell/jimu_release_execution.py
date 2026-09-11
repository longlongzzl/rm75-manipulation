"""SIM execution gate: original clearance contact, then fully checked return.

Observe native concatenation without rewriting/retargeting its output. The
contact exception is the intersection of two GPU audits, never a sphere removal:
all links versus world except the released target, and non-finger world input
versus the complete world. Both audits keep original self-collision inputs.
"""
from collections import deque
import functools
import inspect
import threading
import numpy as np

from .jimu_return_diagnostics import _state
from .pickplace_curobo_only import CuroboOnlyUnsupported
from .transport_contact import jimu_collision_details
from .pickplace_release_contact import assert_full_state
from .world_only_contact import FINGER_LINKS, world_only_links
from .jimu_execution_stages import guarded_stage, stage_kind


def _path(value):
    path=np.asarray(value,dtype=float)
    if path.ndim!=2 or path.shape[1]!=7 or len(path)<2 or not np.isfinite(path).all():
        raise CuroboOnlyUnsupported('missing/invalid Jimu execution path')
    return path


def _dense(path):
    out=[path[0]]
    for start,end in zip(path[:-1],path[1:]):
        count=max(1,int(np.ceil(np.max(abs(end-start))/.01)))
        out.extend(np.linspace(start,end,count+1)[1:])
    return out


def _detail_suffix(detail,index,joints):
    """Read-only pair naming for a failed audit sample; never changes the verdict.

    The audit already runs under the real world state. This only asks the
    reviewed read-only geometry helpers which link overlaps which obstacle at
    the failing dense index, so a report can name the pair instead of only the
    MotionGen status. Failure to produce details never changes the raise.
    """
    if detail is None:return ''
    try:
        return ' | detail='+repr(detail(index,joints))
    except Exception as exc:
        return f' | detail=unavailable: {type(exc).__name__}: {exc}'


class NativePathParts:
    """Bounded, thread-local identity evidence; no stale/copied path matching."""
    def __init__(self, direct):
        self.local=threading.local()
        original=direct._concat_joint_paths

        @functools.wraps(original)
        def concat(*paths):
            result=original(*paths)
            if len(paths)==2:
                # Mirror ONLY native duplicate removal for boundary metadata.
                # A changed native layout fails prefix matching at execution.
                prefix=[]
                for q in list(paths[0] or []):
                    q=np.asarray(q,dtype=np.float32).reshape(-1)[:7].copy()
                    if not prefix or not np.allclose(prefix[-1],q,atol=1e-6,rtol=0):
                        prefix.append(q)
                if not hasattr(self.local,'rows'):self.local.rows=deque(maxlen=16)
                self.local.rows.append((result,np.asarray(result,dtype=float).copy(),
                                        np.asarray(prefix,dtype=float).copy()))
            return result
        direct._concat_joint_paths=concat

    def split(self,value):
        rows=getattr(self.local,'rows',())
        found=next((row for row in reversed(rows) if row[0] is value),None)
        if found is None:raise CuroboOnlyUnsupported('native fused-path boundary evidence missing')
        # Consume by identity (array/list equality is not a safe lookup).
        self.local.rows=deque((row for row in rows if row is not found),maxlen=16)
        path=_path(value);snapshot=found[1];prefix=_path(found[2]);size=len(prefix)
        if (not np.array_equal(path,snapshot) or size>=len(path)
                or not np.array_equal(path[:size],prefix)):
            raise CuroboOnlyUnsupported('native fused-path boundary changed')
        # Include the original join edge in the fully checked return section.
        return path[:size],path[size-1:]


def _qualified_world(planner,source):
    assert_full_state(planner)
    state=_state(planner)
    if not state['world_constraint_enabled'] or not state['self_constraint_enabled']:
        raise CuroboOnlyUnsupported('Jimu release constraints are not active')
    objects=list(planner._world.objects)
    if not any(obj.name=='virtual_table_plane' for obj in objects):
        raise CuroboOnlyUnsupported('Jimu release table collider missing')
    targets=[obj for obj in objects if source and obj.name=='scene_obstacle_'+source]
    if len(targets)!=1:raise CuroboOnlyUnsupported('Jimu released source identity missing/ambiguous')
    return targets[0]


def audit_return(planner,path,source,actual_start,detail=None):
    """Independent return: full current world and self, including its entry edge."""
    _qualified_world(planner,source)
    path=_path(path);start=np.asarray(actual_start,dtype=float)
    if start.shape!=(7,) or not np.isfinite(start).all():
        raise CuroboOnlyUnsupported('Jimu return actual start unavailable')
    connector=not np.array_equal(start,path[0])
    if connector:path=np.vstack((start,path))
    dense=_dense(path)
    for index,joints in enumerate(dense):
        valid,status=planner.check_start_state(joints)
        if not valid:
            raise CuroboOnlyUnsupported(f'Jimu return full-world collision at {index}: {status}'
                                        +_detail_suffix(detail,index,joints))
    return dict(clearance_samples=0,return_samples=len(dense),release_contact_samples=0,
        permitted_contact_target=None,permitted_links=[],return_world_exempt_links=[],
        self_collision_input_modified=False,world_filter_calls=0,entry_connector_audited=connector)


def audit_parts(planner,clearance,return_path,source,detail=None):
    """Keep the original pair-local release policy, with no new depth tolerance."""
    target=_qualified_world(planner,source);dense=_dense(_path(clearance));invalid=[]
    for index,joints in enumerate(dense):
        valid,status=planner.check_start_state(joints)
        if not valid:invalid.append((index,joints))
    filter_calls=0
    if invalid:
        try:
            changed=planner.set_world_obstacles_enabled([target.name],enabled=False)
            if target.name not in changed:
                raise CuroboOnlyUnsupported('Jimu released target scope unavailable')
            for index,joints in invalid:
                valid,status=planner.check_start_state(joints)
                if not valid:
                    raise CuroboOnlyUnsupported(f'Jimu release other-world/self collision at {index}: {status}')
        finally:
            planner.set_world_obstacles_enabled([target.name],enabled=True)
        with world_only_links(planner,FINGER_LINKS,allowed_disabled_objects={'active_target_object'}) as evidence:
            for index,joints in invalid:
                valid,status=planner.check_start_state(joints)
                if not valid:
                    raise CuroboOnlyUnsupported(f'Jimu release non-finger/self collision at {index}: {status}')
            filter_calls=evidence['world_filter_calls']
            if not filter_calls:raise CuroboOnlyUnsupported('Jimu release GPU world filter not observed')
    returning=[] if return_path is None else _dense(_path(return_path))
    for index,joints in enumerate(returning):
        valid,status=planner.check_start_state(joints)
        if not valid:
            raise CuroboOnlyUnsupported(f'Jimu return full-world collision at {index}: {status}'
                                        +_detail_suffix(detail,index,joints))
    return dict(clearance_samples=len(dense),return_samples=len(returning),
                release_contact_samples=len(invalid),permitted_contact_target=target.name,
                permitted_links=sorted(FINGER_LINKS),return_world_exempt_links=[],
                self_collision_input_modified=False,world_filter_calls=filter_calls)


def install_release_execution_guard(portable,emit):
    """Install BEFORE the released-model observer, which must wrap this gate."""
    direct=portable.direct;parts=NativePathParts(direct);records=[]
    direct._jimu_release_execution_audits=records
    original=portable._jimu_execute_pose_path_stage_base;signature=inspect.signature(original)

    @functools.wraps(original)
    def execute(*args,**kwargs):
        bound=signature.bind(*args,**kwargs);bound.apply_defaults();values=bound.arguments
        label=str(values['label']);options=values['args']
        if (not guarded_stage(label) or values['real_exec'] is not None
                or getattr(options,'execute_real',False) or getattr(options,'_planning_prefetch_capture_only',False)):
            return original(*args,**kwargs)
        kind=stage_kind(label)
        row=dict(event='jimu_release_execution_audit',step_id=label,stage_kind=kind,execution_entry='pose',execution_guard=True,
                 passed=False,physical_success=None)
        with direct._CUROBO_GPU_LOCK:
            planner=getattr(values['demo'].planner,'native',None);before=None
            try:
                if planner is None or not getattr(options,'_episode_place_released',False):
                    raise CuroboOnlyUnsupported('Jimu execution gate requires released bound SIM state')
                before=_state(planner);demo=values['demo']
                q=demo.robot.get_qpos()
                if hasattr(q,'detach'):q=q.detach().cpu().numpy()
                q=np.asarray(q,dtype=float).reshape(-1);names=list(demo.active_joint_names)
                locks=before['gripper_lock_joints']
                if (not locks or len(q)!=len(names) or len(set(names))!=len(names)
                        or not np.isfinite(q).all() or not set(locks)<=set(names)
                        or any(locks[name]!=float(q[names.index(name)]) for name in locks)):
                    raise CuroboOnlyUnsupported('Jimu execution gate requires synchronized gripper model')
                source=direct._current_source_object_name(options)
                detail=lambda index,joints:(dict(dense_index=index,source=source,
                    **jimu_collision_details(portable,planner,joints)))
                if kind=='return_only':
                    evidence=audit_return(planner,values['q_path'],source,demo.current_arm_qpos(),
                                          detail=detail)
                elif kind=='release_then_return':
                    clearance,returning=parts.split(values['q_path'])
                    evidence=audit_parts(planner,clearance,returning,source,detail=detail)
                elif kind=='release_only':
                    evidence=audit_parts(planner,_path(values['q_path']),None,source,detail=detail)
                else:raise CuroboOnlyUnsupported('unknown Jimu release/return execution stage')
                row.update(source=source,scene_fingerprint=before['scene_fingerprint'],
                           **evidence,passed=True)
            except BaseException as exc:
                row.update(error_type=type(exc).__name__,error=str(exc))
                if isinstance(exc,Exception):
                    raise CuroboOnlyUnsupported('Jimu release execution audit failed: '+str(exc)) from exc
                raise
            finally:
                row['state_unchanged']=before is not None and _state(planner)==before
                if not row['state_unchanged']:row['passed']=False
                records.append(row);emit(row)
                if before is not None and not row['state_unchanged']:
                    raise CuroboOnlyUnsupported('Jimu release execution audit changed collision state')
        return original(*args,**kwargs)

    portable._jimu_execute_pose_path_stage_base=execute
    return records
