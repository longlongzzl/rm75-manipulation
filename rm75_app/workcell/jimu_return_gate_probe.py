"""Opt-in GPU replay of a native return at a non-moving execution sink.

Does not select/rewrite the actual task path or call its executor. It reuses
the original fused path's return portion with an explicitly injected SIM start.
"""
import functools
import inspect
from types import SimpleNamespace
import numpy as np

from .jimu_release_execution import NativePathParts, install_release_execution_guard
from .jimu_return_diagnostics import _state
from .pickplace_curobo_only import CuroboOnlyUnsupported


def probe_return(direct, demo, options, path, *, evidence=None):
    planner=demo.planner.native;path=np.asarray(path,dtype=float)
    sink_calls=[];gate_rows=[]
    def sink(demo,bridge_mod,real_exec,label,pose,q_path,gripper_pos,args,*,use_attach=False):
        sink_calls.append(label)
        return True,np.asarray(q_path[-1]).copy()
    proxy_direct=SimpleNamespace(_concat_joint_paths=direct._concat_joint_paths,
        _CUROBO_GPU_LOCK=direct._CUROBO_GPU_LOCK,_current_source_object_name=direct._current_source_object_name)
    proxy=SimpleNamespace(direct=proxy_direct,_jimu_execute_pose_path_stage_base=sink)
    install_release_execution_guard(proxy,gate_rows.append)
    proxy_demo=SimpleNamespace(planner=demo.planner,robot=demo.robot,active_joint_names=demo.active_joint_names,
                              current_arm_qpos=lambda:path[0].copy())
    rows=[];evidence={} if evidence is None else evidence
    evidence.update(cases=rows,negative_native_status=None,negative_state_queries=0,
        reused_gpu_planned_return=True,injected_sim_start=True,actual_execute_calls=0,
        fresh_planner_chain=False,physical_success=None)
    def trial(name,label,trial_path,expected_rejected):
        start=len(sink_calls);start_audits=len(gate_rows);rejected=False;error_type=None
        try:
            proxy._jimu_execute_pose_path_stage_base(proxy_demo,None,None,label,None,trial_path,0.,options)
        except CuroboOnlyUnsupported as exc:
            rejected=True;error_type=type(exc).__name__
        audits=gate_rows[start_audits:];count=len(sink_calls)-start
        passed=(rejected==expected_rejected and count==(0 if expected_rejected else 1)
                and len(audits)==1 and audits[0]['state_unchanged']
                and audits[0]['passed'] is (not expected_rejected))
        rows.append(dict(case=name,label=label,passed=passed,rejected=rejected,expected_rejected=expected_rejected,
                         non_motion_sink_calls=count,actual_execute_calls=0,error_type=error_type,
                         execution_audit=audits[0] if len(audits)==1 else None))
        if not passed:raise CuroboOnlyUnsupported('independent return GPU probe did not meet its expected outcome')
    for label in ('return_to_cycle_start','post_place_direct_return_to_cycle_start'):
        trial(label,label,path,False)
    # Deliberate negative input, NEVER a planner candidate or executed path.
    # Find a finite in-limits configuration whose ORIGINAL GPU check reports
    # world/self collision, not merely a joint-limit error.
    limits=planner.robot_cfg.kinematics.kinematics_config.joint_limits.position
    if hasattr(limits,'detach'):limits=limits.detach().cpu().numpy()
    limits=np.asarray(limits,dtype=float)
    if limits.shape!=(2,7):raise CuroboOnlyUnsupported('return probe joint limits unavailable')
    bad=None;queries=0;status=None
    for shoulder in (0.,1.5,-1.5):
        for elbow in (2.8,-2.8,2.2,-2.2):
            candidate=np.clip(np.array([0.,shoulder,0.,elbow,0.,0.,0.]),limits[0],limits[1])
            valid,native_status=planner.check_start_state(candidate);queries+=1
            evidence['negative_state_queries']=queries
            if not valid and any(word in str(native_status) for word in ('WORLD_COLLISION','SELF_COLLISION')):
                bad=candidate;status=str(native_status);break
        if bad is not None:break
    if bad is None:raise CuroboOnlyUnsupported('return probe found no qualified GPU collision negative')
    broken=np.insert(path,max(1,len(path)//2),bad,axis=0)
    trial('injected_colliding_configuration','return_to_cycle_start',broken,True)
    evidence['negative_native_status']=status
    return evidence


def install_return_gate_probe(portable,emit):
    """Validation runner only: install between the production gate and observer."""
    direct=portable.direct;parts=NativePathParts(direct)
    original=portable._jimu_execute_pose_path_stage_base;signature=inspect.signature(original)
    rows=[]
    @functools.wraps(original)
    def execute(*args,**kwargs):
        bound=signature.bind(*args,**kwargs);bound.apply_defaults();values=bound.arguments
        options=values['args']
        if (rows or values['label']!='post_place_clearance_return_to_cycle_start'
                or values['real_exec'] is not None or getattr(options,'execute_real',False)
                or getattr(options,'_planning_prefetch_capture_only',False)):
            return original(*args,**kwargs)
        with direct._CUROBO_GPU_LOCK:
            planner=values['demo'].planner.native;before=_state(planner)
            row=dict(event='jimu_independent_return_gate_probe',passed=False)
            try:
                _,returning=parts.split(values['q_path'])
                probe_return(direct,values['demo'],options,returning,evidence=row)
                row['passed']=True
            except BaseException as exc:
                row.update(error_type=type(exc).__name__,error=str(exc));raise
            finally:
                row['state_unchanged']=_state(planner)==before
                if not row['state_unchanged']:row['passed']=False
                rows.append(row);emit(row)
                if not row['state_unchanged']:
                    raise CuroboOnlyUnsupported('return gate probe changed original collision state')
        return original(*args,**kwargs)
    portable._jimu_execute_pose_path_stage_base=execute
    return rows
