"""Static views of every observed native IK goal, never executed trajectories.

The solver, candidates and return objects are untouched. Only the SIM robot is
temporarily posed; no env.step or native sync helper is called. Purple geometry
is the original URDF tool at the requested pose, black robot is returned IK.
"""
import functools
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .io import atomic_json
from .contact_audit import _plain
from .jimu_roof_ik_diagnostics import result_rows
from .pickplace_lift_diagnostics import _array, _state, _configuration_evidence
from .transforms import quaternion_matrix, rotation_error


def query_context(frame, goals):
    """Identify the original caller, through any diagnostic wrapper layers."""
    while frame is not None:
        v=frame.f_locals; name=frame.f_code.co_name
        if name=='_fast_chain_evaluate_paired_relation_records':
            records=v['records'];n=len(records)
            if len(goals)!=2*n:raise ValueError('Paired gallery goal count mismatch')
            return v['demo'],[(phase,rec['grasp_candidate'],rec['place_candidate'])
                for phase in ('hover','release') for rec in records]
        if name=='_fast_chain_preselect_grasp_place_pair':
            phase='pregrasp' if goals is v.get('pregrasp_poses') else 'grasp' if goals is v.get('grasp_poses') else None
            if phase:return v['demo'],[(phase,c,None) for c in v['candidates']]
        frame=frame.f_back
    return None


def require_gallery(spec, profile):
    if (spec.get('task')!='pickplace' or spec.get('mode')!='sim'
            or profile.get('pickplace',{}).get('fixed_scene_format')!='native_world'):
        raise ValueError('IK gallery requires frozen native-world PickPlace SIM')


def remove_visual(scene, actor):
    """Remove rendering-only actors from both scene and state serialization."""
    actor.remove_from_scene()
    scene.actors.pop(actor.name,None)
    scene.remove_from_state_dict_registry(actor)


def build_goal_tool(demo, planner, name):
    import sapien
    from transforms3d.quaternions import mat2quat
    from rm75_app.planning.gripper_collision import gripper_link_transforms, _rpy_matrix
    cfg=planner.robot_cfg_dict.get('robot_cfg',planner.robot_cfg_dict)['kinematics']
    locks=cfg['lock_joints'];positions=list(locks.values())
    if not positions or max(positions)-min(positions)>1e-6:
        raise ValueError('Gallery requires the original coupled-jaw lock contract')
    urdf=Path(cfg['urdf_path']);transforms=gripper_link_transforms(urdf,positions[0])
    tcp_inv=np.linalg.inv(transforms['gripper_tcp'])
    builder=demo.env.unwrapped.scene.create_actor_builder()
    material=sapien.render.RenderMaterial(base_color=[.85,.2,.85,.40])
    builder.set_initial_pose(sapien.Pose([0,0,-10]))
    for link in ET.parse(urdf).getroot().findall('link'):
        if link.get('name') not in transforms:continue
        for visual in link.findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None:continue
            local=np.eye(4);origin=visual.find('origin')
            if origin is not None:
                local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                local[:3,:3]=_rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' '))
            local=tcp_inv@transforms[link.get('name')]@local
            path=(urdf.parent/mesh.get('filename')).resolve()
            builder.add_visual_from_file(str(path),pose=sapien.Pose(local[:3,3],mat2quat(local[:3,:3])),
                scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ').tolist(),material=material)
    # No collision geometry on the ghost.
    return builder.build_kinematic(name=name),locks


def install(direct, directory, emit):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    original=direct._profile_fast_chain_solve_batch_start_goal_ik
    batches=[];errors=[];counter=0

    @functools.wraps(original)
    def query(options,planner,starts,goals,*,num_seeds):
        nonlocal counter
        with direct._CUROBO_GPU_LOCK:
            results=original(options,planner,starts,goals,num_seeds=num_seeds)
            if getattr(options,'execute_real',False):return results
            context=query_context(sys._getframe(1),goals)
            if context is None:return results
            demo,relations=context
            if len(relations)!=len(results):raise ValueError('Gallery candidate identity mismatch')
            # Copy every raw result before FK/diagnostics touch GPU scratch buffers.
            copied=[result_rows(r,i,len(goals),starts[i]) for i,r in enumerate(results)]
            counter+=1;folder=directory/f'batch_{counter:03d}';folder.mkdir()
            before=_state(planner);saved_q=_array(demo.robot.get_qpos()).copy();saved_v=_array(demo.robot.get_qvel()).copy()
            source=direct._current_source_object_name(options)
            scene=demo.env.unwrapped.scene
            registry_before=set(scene.state_dict_registry.actors)
            actors_before=set(scene.actors)
            batch=dict(batch=counter,source=source,prefetch=bool(getattr(options,'_planning_prefetch_capture_only',False)),
                goal_count=len(goals),num_seeds=num_seeds,rows=[],new_solver_calls=0,physics_steps=0,
                world_state=before,object_world_pose=_plain(demo.get_obj_pose()))
            ghost=None
            try:
                import sapien
                from transforms3d.quaternions import mat2quat
                from mani_skill.utils import sapien_utils
                from PIL import Image,ImageDraw,ImageFont
                font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
                small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
                ghost,locks=build_goal_tool(demo,planner,f'ik_gallery_goal_{counter}')
                names=[joint.name for joint in demo.robot.get_active_joints()]
                base=np.asarray(direct._get_robot_base_world_transform(demo),dtype=float)
                for i,(result,goal,(phase,grasp,place),(seeds,nearest)) in enumerate(zip(results,goals,relations,copied)):
                    q=np.asarray(result.goal_joint if result.success and result.goal_joint is not None else seeds[nearest]['joints'])
                    if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Nonfinite gallery IK row')
                    configuration=_configuration_evidence(planner,q)
                    goal_base=np.asarray(direct._pose_to_matrix_from_pose_obj(goal),dtype=float)
                    target=base@goal_base
                    posed=saved_q.copy().reshape(-1);posed[demo.arm_indices]=q
                    for name,value in locks.items():posed[names.index(name)]=value
                    demo.robot.set_qpos(posed.reshape(saved_q.shape))
                    ghost.set_pose(sapien.Pose(target[:3,3],mat2quat(target[:3,:3])))
                    actual=np.asarray(direct._pose_to_matrix_from_pose_obj(demo.tcp.pose),dtype=float)
                    center=(target[:3,3]+actual[:3,3])/2
                    base_renderer=direct.targeted.base
                    overview=base_renderer.capture_failure_render_image(demo,options)
                    detail=base_renderer.capture_failure_render_image(demo,options,
                        camera_pose=sapien_utils.look_at(center+np.array([.32,-.32,.23]),center))
                    top=base_renderer.capture_failure_render_image(demo,options,
                        camera_pose=sapien_utils.look_at(center+np.array([.32,.32,.14]),center))
                    if any(x is None for x in (overview,detail,top)):raise RuntimeError('Missing native candidate render')
                    pos_error=float(np.linalg.norm(actual[:3,3]-target[:3,3]))
                    angle=rotation_error(actual[:3,:3],target[:3,:3])
                    row=dict(index=i,phase=phase,grasp_label=grasp.get('label'),
                        place_label=place.get('label') if place else None,
                        relation_transform_tcp_object=_plain(grasp.get('T_tcp_obj')),
                        goal_world=_plain(target),actual_tcp_world=_plain(actual),
                        returned_seeds=seeds,rendered_q=q.tolist(),native_success=bool(result.success),
                        status=str(result.status),position_error_m=pos_error,so3_error_rad=angle,
                        configuration=configuration,image=f'batch_{counter:03d}/candidate_{i:03d}.jpg')
                    row['grasp_pregrasp_accepted']=grasp.get('_winner_preselect_pregrasp_success')
                    row['grasp_approach_accepted']=grasp.get('_winner_preselect_grasp_success')
                    image=Image.new('RGB',(1536,620),(20,25,32));draw=ImageDraw.Draw(image)
                    ok='IK PASS' if row['native_success'] else 'IK FAIL / NOT EXECUTED'
                    draw.text((12,8),f'{source} | batch {counter} | {phase} #{i} | {ok} | '+('PREFETCH' if batch['prefetch'] else 'FOREGROUND'),font=font,fill='white')
                    draw.text((12,32),str(row['grasp_label'])+' / '+str(row['place_label'] or ''),font=small,fill='white')
                    draw.text((12,54),f'Purple = requested tool pose. Black robot = returned IK. Position error {pos_error*1000:.2f} mm / angle {np.degrees(angle):.2f} deg.',font=small,fill=(255,210,120))
                    draw.text((12,76),'STATIC INSPECTION ONLY. Scene objects remain at current poses; no carried-object teleport. IK PASS is not a valid full path.',font=small,fill='white')
                    for k,label in enumerate(('Whole scene','Tool close-up','Opposite side / pad-plane view')):
                        draw.text((k*512+12,94),label,font=small,fill='white')
                    for k,pixels in enumerate((overview,detail,top)):
                        image.paste(Image.fromarray(pixels).convert('RGB').resize((512,512)),(k*512,108))
                    image.save(directory/row['image'],quality=88)
                    batch['rows'].append(row)
            except Exception as exc:
                batch['render_error']=f'{type(exc).__name__}: {exc}';errors.append(batch['render_error'])
            finally:
                demo.robot.set_qpos(saved_q);demo.robot.set_qvel(saved_v)
                if ghost is not None:
                    remove_visual(scene,ghost)
                batch['robot_state_restored']=bool(np.array_equal(_array(demo.robot.get_qpos()),saved_q)
                    and np.array_equal(_array(demo.robot.get_qvel()),saved_v))
                batch['planner_state_restored']=_state(planner)==before
                batch['actor_registry_restored']=set(scene.state_dict_registry.actors)==registry_before and set(scene.actors)==actors_before
                batch['returned_rows_unchanged']=[result_rows(r,i,len(goals),starts[i]) for i,r in enumerate(results)]==copied
                atomic_json(folder/'evidence.json',batch);batches.append(batch)
                emit(dict(event='ik_candidate_gallery_batch',batch=counter,source=source,goal_count=len(goals),
                    rendered=len(batch['rows']),prefetch=batch['prefetch'],render_error=batch.get('render_error')))
                if not all(batch[k] for k in ('robot_state_restored','planner_state_restored','actor_registry_restored','returned_rows_unchanged')):
                    raise RuntimeError('IK gallery failed exact state/result restoration')
            return results
    direct._profile_fast_chain_solve_batch_start_goal_ik=query
    def close():
        direct._profile_fast_chain_solve_batch_start_goal_ik=original
        atomic_json(directory/'manifest.json',dict(schema='rm75.ik_candidate_gallery/v1',
            batches=[{k:v for k,v in b.items() if k not in ('rows','world_state')} for b in batches],
            complete=bool(batches) and not errors and all(len(b['rows'])==b['goal_count'] for b in batches),
            errors=errors,execute_real=False,hardware_connected=False))
    return close
