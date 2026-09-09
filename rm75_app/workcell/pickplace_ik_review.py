"""Explicit frozen Tennis contract checks and fixed-budget IK reference trial.

No vendor edits, target translations or collision exemptions. The separately
labelled yaw-midpoints experiment appends bounded horizontal axial angles only.
Raw relation/seed records are emitted only to the local worker evidence log.
"""
import contextvars
import copy
import functools
import inspect
import time
import numpy as np

from .contact_audit import _plain
from .pickplace_lift_diagnostics import _array, _state, _configuration_evidence
from .jimu_roof_ik_diagnostics import result_rows
from .transforms import quaternion_matrix, rotation_error


def center_closure(transform, tcp_matrix, desired_center):
    relation=np.asarray(transform,dtype=float).reshape(4,4)
    tcp=np.asarray(tcp_matrix,dtype=float).reshape(4,4)
    wanted=np.asarray(desired_center,dtype=float).reshape(3)
    predicted=tcp[:3,3]+tcp[:3,:3]@relation[:3,3]
    if not np.isfinite([*predicted,*wanted]).all():raise ValueError('Nonfinite sphere relation')
    return dict(translation_tcp=relation[:3,3].tolist(),desired_center=wanted.tolist(),
        predicted_center=predicted.tolist(),center_residual_m=float(np.linalg.norm(predicted-wanted)),
        lateral_tcp_offset_m=float(np.linalg.norm(relation[:2,3])),signed_axial_offset_m=float(relation[2,3]))


def fix_selected_debug(result):
    """Keep chosen q and its own raw errors together; never promote success."""
    if not result.success or result.goal_joint is None:return result
    raw=result.raw_result
    q=_array(raw.solution).reshape(-1,7);mask=_array(raw.success).reshape(-1).astype(bool)
    candidates=np.flatnonzero(mask)
    if not len(candidates):raise ValueError('Selected IK has no successful raw row')
    index=int(candidates[np.argmin(np.linalg.norm(q[candidates]-np.asarray(result.goal_joint),axis=1))])
    if np.linalg.norm(q[index]-result.goal_joint)>1e-6:raise ValueError('Selected IK row identity mismatch')
    result.debug=dict(result.debug or {},selected_raw_index=index,
        position_error=float(_array(raw.position_error).reshape(-1)[index]),
        rotation_error=float(_array(raw.rotation_error).reshape(-1)[index]))
    return result


def yaw_midpoints(original):
    angles=sorted({float(value)%360 for value in original})
    added=[(a+((angles[(i+1)%len(angles)]-a)%360)/2)%360
           for i,a in enumerate(angles)]
    return list(original)+[a if a<=180 else a-360 for a in added if a not in angles]


def install(direct, planner_class, emit, *, strategy):
    if strategy not in ('baseline','standard-reference','continuation','yaw-midpoints'):raise ValueError('Unknown Tennis IK review strategy')
    originals=[];context=contextvars.ContextVar('tennis_ik_review',default=None);roundtrip_done=False
    def patch(owner,name,replacement):
        originals.append((owner,name,getattr(owner,name)));setattr(owner,name,replacement)
    if strategy=='yaw-midpoints':
        original_angles=direct._tennis_release_axial_roll_degs
        def augmented(options):
            original=list(original_angles(options));angles=yaw_midpoints(original)
            emit(dict(event='tennis_horizontal_yaw_coverage',original_degrees=original,
                augmented_degrees=angles,original_angles_preserved=True))
            return angles
        patch(direct,'_tennis_release_axial_roll_degs',augmented)
    convert=direct._convert_demo_tcp_pose_to_curobo_ee_pose
    @functools.wraps(convert)
    def checked_convert(demo,pose,*,ee_link_name):
        if direct._get_robot_link_by_name(demo,ee_link_name) is None:
            raise ValueError('Missing actual cuRobo EE link: '+ee_link_name)
        return convert(demo,pose,ee_link_name=ee_link_name)
    patch(direct,'_convert_demo_tcp_pose_to_curobo_ee_pose',checked_convert)
    solve=planner_class.solve_ik
    @functools.wraps(solve)
    def selected(self,*args,**kwargs):return fix_selected_debug(solve(self,*args,**kwargs))
    patch(planner_class,'solve_ik',selected)

    def matrix(pose):return direct._pose_to_matrix_from_pose_obj(pose).astype(float)
    def pose_errors(planner,q,goal):
        actual=planner.fk(q)
        return dict(position_error_m=float(np.linalg.norm(np.asarray(actual['position'])-goal['position'])),
            so3_error_rad=rotation_error(quaternion_matrix(actual['quaternion']),quaternion_matrix(goal['quaternion'])))
    query=direct._profile_fast_chain_solve_batch_start_goal_ik
    @functools.wraps(query)
    def reviewed_query(options,planner,starts,goals,*,num_seeds):
        nonlocal roundtrip_done
        current=context.get()
        if current is None:return query(options,planner,starts,goals,num_seeds=num_seeds)
        if num_seeds!=32:raise ValueError('Reviewed trial must preserve 32 IK seeds')
        demo,records,start_q,label=current
        with direct._CUROBO_GPU_LOCK:
            before=_state(planner);tick=time.perf_counter()
            if not roundtrip_done:
                roundtrip_done=True
                valid_q=next((np.asarray(q,dtype=float) for q in [start_q,*starts]
                              if planner.check_start_state(q)[0]),None)
                if valid_q is None:raise ValueError('No current-world-valid original q for FK/IK roundtrip')
                goal=planner.fk(valid_q);state=planner._make_start_state(valid_q)
                rows=[]
                for method in ('solve_single','solve_batch','frontend_wrapper'):
                    planner.ik_solver.reset_seed()
                    if method=='frontend_wrapper':
                        result=query(options,planner,[valid_q],[goal],num_seeds=32)[0]
                        success=bool(result.success);q=result.goal_joint
                    else:
                        result=getattr(planner.ik_solver,method)(planner._make_pose(goal),
                            retract_config=state.position.clone(),seed_config=state.position.reshape(1,1,7).clone(),
                            num_seeds=32,return_seeds=1,use_nn_seed=False)
                        success=bool(_array(result.success).reshape(-1)[0]);q=_array(result.solution).reshape(-1,7)[0]
                    rows.append(dict(method=method,success=success,
                        **(pose_errors(planner,q,goal) if q is not None else {})))
                ee_name=planner.config.ee_link
                link=direct._get_robot_link_by_name(demo,ee_name)
                if link is None:raise ValueError('Missing actual cuRobo EE link: '+ee_name)
                base=direct._get_robot_base_world_transform(demo)
                tcp_to_ee=np.linalg.inv(matrix(demo.tcp.pose))@matrix(link.pose)
                target_base=np.eye(4);target_base[:3,3]=goal['position'];target_base[:3,:3]=quaternion_matrix(goal['quaternion'])
                target_tcp=base@target_base@np.linalg.inv(tcp_to_ee)
                recon=matrix(checked_convert(demo,direct.targeted._pose_from_matrix(target_tcp),ee_link_name=ee_name))
                emit(dict(event='tennis_fk_ik_contract',strategy=strategy,roundtrips=rows,
                    ee_link=ee_name,joint_names=list(planner.joint_names),joint_units='radians',quaternion_order='wxyz',
                    T_world_base=_plain(base),T_demo_tcp_ee=_plain(tcp_to_ee),
                    conversion_matrix_max_error=float(np.max(abs(recon-target_base))),
                    state_unchanged=_state(planner)==before))
            trial_started=time.perf_counter()
            original_batch=planner.ik_solver.solve_batch
            standard=planner.ik_solver.get_retract_config().reshape(1,7)
            continuation_seeds={}
            if strategy=='continuation':
                # A bounded seed-only search near existing endpoints. Original
                # targets, orientations, grasp seeds, reference q and 32 total
                # seeds are retained. No helper pose/path enters execution.
                helper_goals=[];helper_starts=[];identities=[];seen={}
                for index,goal in enumerate(goals):
                    pose=planner._pose_to_dict(planner._make_pose(goal))
                    key=tuple(np.round([*pose['position'],*pose['quaternion']],5))
                    if key in seen:continue
                    seen[key]=index;p=np.asarray(pose['position'],dtype=float)
                    inward=-p.copy();inward[2]=0;inward/=max(np.linalg.norm(inward),1e-9)
                    for radial,up in ((0.,.04),(.04,0.),(.04,.04),(.08,.06)):
                        shifted=dict(position=(p+inward*radial+[0,0,up]).tolist(),quaternion=pose['quaternion'])
                        helper_goals.append(shifted);helper_starts.append(starts[index]);identities.append((key,index,shifted))
                helper_results=planner.solve_batch_start_goal_ik(helper_starts,helper_goals,num_seeds=32,use_cuda_graph_batch=False)
                counts=[sum(bool(r.success) for r in helper_results)]
                live=[(identities[i],r.goal_joint) for i,r in enumerate(helper_results) if r.success]
                for fraction in (.25,.5,.75,1.):
                    if not live:break
                    targets=[];first=[];second=[]
                    for (key,index,shifted),q in live:
                        target=planner._pose_to_dict(planner._make_pose(goals[index]))
                        target['position']=((1-fraction)*np.asarray(shifted['position'])+fraction*np.asarray(target['position'])).tolist()
                        targets.append(target);first.append(starts[index]);second.append(q)
                    def seeded_batch(goal,*args,**kwargs):
                        extra=planner._make_multi_start_state(second).position.reshape(len(second),1,7)
                        kwargs['seed_config']=planner.mods['torch'].cat((kwargs['seed_config'],extra),dim=1)
                        return original_batch(goal,*args,**kwargs)
                    planner.ik_solver.solve_batch=seeded_batch
                    try:refined=planner.solve_batch_start_goal_ik(first,targets,num_seeds=32,use_cuda_graph_batch=False)
                    finally:planner.ik_solver.solve_batch=original_batch
                    live=[(item[0],r.goal_joint) for item,r in zip(live,refined) if r.success]
                    counts.append(len(live))
                for (key,index,_),q in live:continuation_seeds.setdefault(key,q)
                emit(dict(event='tennis_continuation_seed_search',helper_goal_count=len(helper_goals),
                    successful_by_fraction=counts,final_endpoint_seed_count=len(continuation_seeds),
                    num_seeds_per_goal=32,helpers_are_execution_paths=False,state_unchanged=_state(planner)==before))
            if strategy=='standard-reference':
                def batch(goal,*args,**kwargs):
                    kwargs['retract_config']=standard.repeat(kwargs['seed_config'].shape[0],1).clone()
                    return original_batch(goal,*args,**kwargs)
                planner.ik_solver.solve_batch=batch
            elif strategy=='continuation' and continuation_seeds:
                extra=[]
                for index,goal in enumerate(goals):
                    pose=planner._pose_to_dict(planner._make_pose(goal))
                    key=tuple(np.round([*pose['position'],*pose['quaternion']],5))
                    extra.append(continuation_seeds.get(key,starts[index]))
                def batch(goal,*args,**kwargs):
                    secondary=planner._make_multi_start_state(extra).position.reshape(len(extra),1,7)
                    kwargs['seed_config']=planner.mods['torch'].cat((kwargs['seed_config'],secondary),dim=1)
                    return original_batch(goal,*args,**kwargs)
                planner.ik_solver.solve_batch=batch
            try:results=query(options,planner,starts,goals,num_seeds=num_seeds)
            finally:planner.ik_solver.solve_batch=original_batch
            solve_s=time.perf_counter()-trial_started
            saved=[]
            for i,(result,seed,goal) in enumerate(zip(results,starts,goals)):
                raw_rows,_=result_rows(result,i,len(goals),seed)
                q=raw_rows[0]['joints']
                target=dict(position=_plain(goal.p),quaternion=_plain(goal.q)) if hasattr(goal,'p') else goal
                target={k:np.asarray(v).reshape(-1).tolist() for k,v in target.items() if k in ('position','quaternion')}
                saved.append(dict(goal_index=i,endpoint='hover' if i<len(records) else 'release',
                    explicit_grasp_seed=_plain(seed),reference_q=_plain(standard.reshape(-1) if strategy=='standard-reference' else seed),
                    goal=target,raw_rows=raw_rows,**pose_errors(planner,q,target),
                    configuration=_configuration_evidence(planner,q)))
            emit(dict(event='tennis_paired_ik_trial',strategy=strategy,step_id=label,relations=len(records),
                requested_num_seeds=num_seeds,goal_count=len(goals),native_success_count=sum(bool(r.success) for r in results),
                solve_wall_s=solve_s,diagnostic_and_solve_wall_s=time.perf_counter()-tick,
                goals=saved,state_before=before,state_unchanged=_state(planner)==before))
            return results
    patch(direct,'_profile_fast_chain_solve_batch_start_goal_ik',reviewed_query)
    paired=direct._fast_chain_evaluate_paired_relation_records;signature=inspect.signature(paired)
    @functools.wraps(paired)
    def relations(*args,**kwargs):
        v=signature.bind(*args,**kwargs).arguments;options=v['args']
        if direct._current_source_object_name(options)!='tennis':return paired(*args,**kwargs)
        if getattr(options,'execute_real',False):raise ValueError('Tennis IK review is frozen SIM only')
        records=v['records'];rows=[]
        for i,record in enumerate(records):
            grasp=record['grasp_candidate'];place=record['place_candidate'];plan=place['place_plan']
            rows.append(dict(relation_index=i,grasp_label=grasp.get('label'),place_label=place.get('label'),
                **center_closure(grasp['T_tcp_obj'],matrix(place['release_pose']),
                                 np.asarray(plan.T_world_obj_desired)[:3,3])))
        emit(dict(event='tennis_sphere_center_contract',strategy=strategy,relations=rows))
        token=context.set((v['demo'],records,v['start_q'],kwargs.get('label','paired')))
        try:return paired(*args,**kwargs)
        finally:context.reset(token)
    patch(direct,'_fast_chain_evaluate_paired_relation_records',relations)
    def close():
        for owner,name,original in reversed(originals):setattr(owner,name,original)
    return close
