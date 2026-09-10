"""Opt-in first roof IK batch per phase/source, separately for prefetch/foreground.

Install BEFORE grasp-contact scoping so diagnostics see that query's scope.
No extra solve, result promotion, candidate filtering, geometry or mask change.
"""
import functools
import threading
import numpy as np

from .contact_audit import _plain
from .pickplace_lift_diagnostics import _array, _state, _configuration_evidence
from .pickplace_curobo_only import CuroboOnlyUnsupported
from .pickplace_clearance_audit import sphere_box_contacts
from .transforms import quaternion_matrix


def roof_audit_status(rows,expected_sources):
    """A requested audit cannot pass by observing nothing or omitting a phase."""
    expected={(source,phase) for source in expected_sources
              for phase in ('pregrasp','grasp','paired_place')}
    observed={(row.get('source'),row.get('phase')) for row in rows}
    missing=expected-observed
    complete=bool(rows) and all(row.get('diagnostic_complete') is True
                               and row.get('state_unchanged') is True for row in rows)
    return dict(passed=bool(expected and not missing and complete),observed_batches=len(rows),
        observed_batches_complete=complete,
        prefetch_batches=sum(row.get('prefetch') is True for row in rows),
        foreground_batches=sum(row.get('prefetch') is False for row in rows),
        expected_sources=sorted(set(expected_sources)),
        missing_scopes=[dict(source=s,phase=p) for s,p in sorted(missing)])


def nominal_gripper_base_boxes(planner,reference_q,comparison_q,goals):
    """Necessary box-world condition at the EXACT original EE goals, not new IK.

    Only gripper_base_link is rigidly placed. Arm/self, mesh obstacles, payload,
    and physical geometry qualification are explicitly outside this check.
    """
    link='gripper_base_link';links=planner._collision_sphere_link_names()
    if link in planner._disabled_collision_links:raise ValueError('gripper base link disabled')
    spheres=np.asarray(planner._compute_world_link_spheres(reference_q),dtype=float)
    comparison=np.asarray(planner._compute_world_link_spheres(comparison_q),dtype=float)
    if (spheres.shape!=(len(links),4) or comparison.shape!=spheres.shape
            or not np.isfinite(spheres).all() or not np.isfinite(comparison).all()):
        raise ValueError('invalid gripper base sphere layout')
    indices=[i for i,name in enumerate(links) if name==link and spheres[i,3]>0]
    if not indices:raise ValueError('gripper base spheres missing')
    def relative(values,q):
        pose=planner.fk(q)
        return (values[indices,:3]-np.asarray(pose['position']))@quaternion_matrix(pose['quaternion'])
    local=relative(spheres,reference_q)
    delta=float(max(np.max(np.abs(relative(comparison,comparison_q)-local)),
                    np.max(np.abs(comparison[indices,3]-spheres[indices,3]))))
    if not np.isfinite(delta) or delta>1e-6:raise ValueError('gripper base geometry is not EE-rigid')
    rows=[]
    for index,goal in enumerate(goals):
        nominal=spheres[indices].copy()
        nominal[:,:3]=local@quaternion_matrix(goal['quaternion']).T+np.asarray(goal['position'])
        if not np.isfinite(nominal).all():raise ValueError('invalid nominal gripper base pose')
        contacts=sphere_box_contacts(nominal,[link]*len(indices),planner._world.objects,
                                    planner._disabled_world_obstacles)
        rows.append(dict(goal_index=index,box_contacts=contacts,
            enabled_box_overlap=any(c['enabled'] for c in contacts)))
    return dict(link=link,sphere_count=len(indices),rigid_comparison_max_delta_m=delta,
        geometric_necessary_condition_only=True,physical_geometry_qualified=False,
        non_box_objects_not_analytically_audited=[obj.name for obj in planner._world.objects
                                               if getattr(obj,'dims',None) is None],goals=rows)


def result_rows(result, goal_index, goal_count, start):
    """Map native raw seed errors and joints without broadcasting one seed's error."""
    # Explicit provenance for bounded, independent single-goal SIM retries.
    # Keep all native seed rows; never pretend they form the original batch.
    if (getattr(result,'debug',{}) or {}).get('ik_search_goal_count') == 1:
        goal_count = 1
    raw=result.raw_result;solution=_array(raw.solution).astype(np.float32,copy=True)
    columns={name:_array(getattr(raw,name)).copy() for name in ('success','position_error','rotation_error')}
    if solution.ndim not in (2,3) or solution.shape[-1]!=7:
        raise ValueError('unsupported raw IK solution layout')
    if any(value.size!=np.prod(solution.shape[:-1]) for value in columns.values()):
        raise ValueError('raw IK solution/error count mismatch')
    columns={name:value.reshape(solution.shape[:-1]) for name,value in columns.items()}
    if goal_count==1:
        solution=solution.reshape(-1,7);columns={key:value.reshape(-1) for key,value in columns.items()}
    else:
        index=(getattr(result,'debug',{}) or {}).get('batch_index',goal_index)
        if type(index) is not int or solution.shape[0]!=goal_count or not 0<=index<goal_count:
            raise ValueError('raw IK batch identity unavailable')
        solution=solution[index].reshape(-1,7)
        columns={key:value[index].reshape(-1) for key,value in columns.items()}
    if not len(solution):raise ValueError('empty raw IK seed rows')
    finite_q=np.isfinite(solution).all(axis=1)
    indices=np.flatnonzero(finite_q)
    if not len(indices):raise ValueError('no finite raw IK seed')
    reference=np.asarray(start,dtype=np.float32).reshape(-1)
    if reference.shape!=(7,) or not np.isfinite(reference).all():raise ValueError('invalid IK start')
    nearest=int(indices[np.argmin(np.linalg.norm(solution[indices]-reference,axis=1))])
    rows=[]
    for index,q in enumerate(solution):
        finite=bool(finite_q[index] and np.isfinite(columns['position_error'][index])
                    and np.isfinite(columns['rotation_error'][index]))
        rows.append(dict(seed_index=index,finite=finite,native_success=bool(columns['success'][index]),
            joints=q.tolist() if finite_q[index] else None,
            position_error_m=float(columns['position_error'][index]) if finite else None,
            rotation_error_native=float(columns['rotation_error'][index]) if finite else None))
    return rows,nearest


def install_roof_ik_diagnostics(direct,emit,*,sources=None,event_name='jimu_roof_ik_batch_diagnostic'):
    local=threading.local();records=[];seen=set()
    original_refresh=direct._refresh_curobo_world
    original_toggle=direct._set_world_collision_for_links
    original_query=direct._profile_fast_chain_solve_batch_start_goal_ik

    @functools.wraps(original_refresh)
    def refresh(planner,demo,args,**kwargs):
        result=original_refresh(planner,demo,args,**kwargs)
        local.world=(id(planner),str(kwargs.get('label','')))
        return result

    @functools.wraps(original_toggle)
    def toggle(planner,links,*,enabled,label):
        result=original_toggle(planner,links,enabled=enabled,label=label)
        if 'paired_relation_ik' in str(label):
            local.paired=None if enabled else (id(planner),str(label))
        return result

    @functools.wraps(original_query)
    def query(options,planner,starts,goals,*,num_seeds):
        with direct._CUROBO_GPU_LOCK:
            # Native starts/goals are sequences; do not consume generators or replace arguments.
            result=original_query(options,planner,starts,goals,num_seeds=num_seeds)
            source=direct._current_source_object_name(options)
            eligible=(source in sources if sources is not None else 'roof_triangle' in str(source))
            if not eligible or getattr(options,'execute_real',False):return result
            prefetch=bool(getattr(options,'_planning_prefetch_capture_only',False))
            world=getattr(local,'world',(None,''));paired=getattr(local,'paired',None)
            if paired and paired[0]==id(planner):phase='paired_place';label=paired[1]
            elif world[0]==id(planner) and ('pregrasp' in world[1] or
                    world[1]=='winner_chain_ik_preselect_grasp'):phase='pregrasp';label=world[1]
            elif world[0]==id(planner) and 'grasp_contact' in world[1]:phase='grasp';label=world[1]
            else:return result
            key=(source,phase,prefetch)
            if key in seen:return result
            seen.add(key)
            row=dict(event=event_name,source=source,phase=phase,step_id=label,
                diagnostic_only=True,execution_guard=False,prefetch=prefetch,requested_num_seeds=num_seeds,
                goal_count=len(goals),native_success_count=sum(bool(item.success) for item in result),
                original_near_position_threshold=getattr(options,'jimu_near_ik_position_threshold',1e-4),
                original_near_rotation_threshold=getattr(options,'jimu_near_ik_rotation_threshold',1e-3),
                goals=[],configuration_details=[])
            before=None
            try:
                before=_state(planner);row.update(before)
                if not len(goals) or len(starts)!=len(goals) or len(result)!=len(goals):
                    raise ValueError('native IK batch count mismatch/empty')
                # Copy the entire returned batch's raw rows before any diagnostic GPU FK/check.
                for index,item in enumerate(result):
                    seeds,nearest=result_rows(item,index,len(goals),starts[index])
                    position,quaternion=planner._extract_pose_components(goals[index])
                    debug=getattr(item,'debug',{}) or {}
                    row['goals'].append(dict(goal_index=index,native_success=bool(item.success),
                        native_status=str(item.status),goal_pose={'position':_plain(position),'quaternion':_plain(quaternion)},
                        debug_position_error_m=debug.get('position_error'),
                        debug_rotation_error_native=debug.get('rotation_error'),
                        legacy_nearest_seed_index=nearest,seeds=seeds))
                failed=[item for item in row['goals'] if not item['native_success']]
                selected=[]
                if failed:
                    selected.append(failed[0])
                    finite=[item for item in failed if item['seeds'][item['legacy_nearest_seed_index']]['finite']]
                    if finite:
                        closest=min(finite,key=lambda item:item['seeds'][item['legacy_nearest_seed_index']]['position_error_m'])
                        if closest is not selected[0]:selected.append(closest)
                for item in selected:
                    seed=item['seeds'][item['legacy_nearest_seed_index']]
                    detail=_configuration_evidence(planner,seed['joints'])
                    actual=np.asarray(detail['fk']['position']);goal=np.asarray(item['goal_pose']['position'])
                    row['configuration_details'].append(dict(goal_index=item['goal_index'],seed_index=seed['seed_index'],
                        independently_measured_position_error_m=float(np.linalg.norm(actual-goal)),**detail))
                if phase=='paired_place':
                    first=row['goals'][0]
                    row['nominal_gripper_base_boxes']=nominal_gripper_base_boxes(planner,starts[0],
                        first['seeds'][first['legacy_nearest_seed_index']]['joints'],
                        [item['goal_pose'] for item in row['goals']])
                row['diagnostic_complete']=all(seed['finite'] for item in row['goals'] for seed in item['seeds'])
            except Exception as exc:
                row.update(diagnostic_complete=False,diagnostic_error=f'{type(exc).__name__}: {exc}')
            finally:
                row['state_unchanged']=before is not None and _state(planner)==before
                records.append(row);emit(row)
                if not row['state_unchanged']:
                    raise CuroboOnlyUnsupported('roof IK diagnostic changed collision state')
            return result
    direct._refresh_curobo_world=refresh
    direct._set_world_collision_for_links=toggle
    direct._profile_fast_chain_solve_batch_start_goal_ik=query
    return records
