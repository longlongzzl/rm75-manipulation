#!/usr/bin/env python3
"""Persistent cuRobo2 planning-only process for explicit physics simulations.

Only joint/pose JSON enters; no SDK, camera, execute_push or hardware profile.
Physics lives in its existing separate Python environment, without installations.
"""
import argparse
import contextlib
from dataclasses import asdict,replace
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.events import StopToken


def audit_prepared_retreat(executor,prepared):
    """Audit a retreat in the caller's stage-correct private scene.

    The legacy generator restores collision checks before returning. Reuse its
    exhaustive native audit on the timed retreat with no allowed target contact.
    A later SWM boundary must still supply the measured post-push scene.
    """
    rows=[path for stage,path,_ in prepared.stages if stage=='retreat']
    if not rows:raise ValueError('Prepared push has no retreat to audit')
    for path in rows:
        try:executor._audit(path,contact=False)
        except Exception as exc:
            executor.events.emit('physics_retreat_native_rejected',
                scope='predicted_post_push_ensemble',samples=len(path),
                start_q=np.asarray(path[0]).tolist(),end_q=np.asarray(path[-1]).tolist(),
                error=f'{type(exc).__name__}: {exc}'[:4096],
                post_push_scene_verified=False)
            raise
    executor.events.emit('physics_retreat_native_audit',
        scope='predicted_post_push_ensemble',samples=sum(len(path) for path in rows),
        post_push_scene_verified=False)


def audit_predicted_retreat(executor,prepared,push,observation):
    """Original response model and original contact binding, private scenes only."""
    from rm75_app.pusht.model import predict
    original_scene=executor._scene(observation)
    bound=replace(push,contact=tuple(executor.last_contact_binding['surface_contact_xyz'][:2]))
    if not executor.config.friction_scales:
        raise ValueError('Retreat requires a nonempty prediction ensemble')
    try:
        for scale in executor.config.friction_scales:
            future=predict(observation.pose,bound,executor.config,scale=scale)
            predicted=replace(observation,pose=tuple(future),source='prediction')
            executor.backend.update_scene(executor._scene(predicted))
            audit_prepared_retreat(executor,prepared)
    finally:
        executor.backend.update_scene(original_scene)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True,type=Path);parser.add_argument('--directory',required=True,type=Path)
    args=parser.parse_args();data=json.loads(args.profile.read_text());wire=sys.stdout
    def send(value):wire.write(json.dumps(value,allow_nan=False)+'\n');wire.flush()
    backend=None
    try:
        with contextlib.redirect_stdout(sys.stderr):
            from rm75_app.planning.backends.curobo2 import Curobo2Backend,Curobo2BackendConfig
            from rm75_app.pusht.motion import CuroboPushExecutor,pusht_planner_options
            class AuditedPhysicsPushExecutor(CuroboPushExecutor):
                def plan_push(self,push,observation,**kwargs):
                    prepared=super().plan_push(push,observation,**kwargs)
                    audit_predicted_retreat(self,prepared,push,observation)
                    return prepared
            from rm75_app.pusht.model import Config,Push
            from rm75_app.pusht.observation import Observation
            from rm75_app.planning.contracts import CollisionObject,Pose,PlanningScene
            options=pusht_planner_options(data.get('planner',{}))
            for key in ('robot_config','curobo_root'):
                if key in options and options[key] is not None:options[key]=Path(options[key])
            backend=Curobo2Backend(Curobo2BackendConfig(**options))
            native=backend._ensure_planner()
            q=native.default_joint_state.position.reshape(-1)[:7].detach().cpu().numpy()
            backend.set_gripper_collision_state(closed=True)
            geometry=backend.closed_gripper_tool_geometry(q)
            periodic_audits=[]
            solve_variants=backend.solve_pose_ik_variants
            def audited_variants(request):
                result=solve_variants(request)
                periodic_audits.append(dict(backend._stage_ik_periodic_audit))
                return result
            backend.solve_pose_ik_variants=audited_variants
            motion_diagnostics=[]
            plan_candidates=backend.plan_candidates
            def recorded_plan(request):
                result=plan_candidates(request)
                row=dict(stage=request.candidates[0].candidate_id,
                    start_q=request.current.positions.tolist(),
                    goals=[asdict(candidate.pose) for candidate in request.candidates],
                    plans=[dict(candidate_id=p.candidate_id,success=p.success,status=p.status,
                        position_error=p.position_error,orientation_error=p.orientation_error,
                        diagnostics=p.diagnostics) for p in result.plans])
                if not any(p.success for p in result.plans):
                    mods=backend._import_modules()
                    state=mods['JointState'].from_position(mods['torch'].as_tensor(
                        request.current.positions,device=native.device_cfg.device,
                        dtype=native.device_cfg.dtype).reshape(1,7),joint_names=list(native.joint_names))
                    row['start_contacts']=backend._collision_diagnostics_for_states(native,{'start':state})
                motion_diagnostics.append(json.loads(json.dumps(row,default=lambda x:x.tolist())))
                return result
            backend.plan_candidates=recorded_plan
            import yaml
            robot_cfg=yaml.safe_load(Path(backend.config.robot_config).read_text())['robot_cfg']['kinematics']
            send(dict(ready=True,retreat_collision_checks=False,ignore_gripper_internal_self_collision=backend.config.ignore_gripper_internal_self_collision,initial_q=q.tolist(),spheres=geometry.spheres.tolist(),links=list(geometry.links),
                urdf=robot_cfg['urdf_path'],
                gripper_locks={name:backend.config.gripper_collision_closed_joint_position
                               for name in robot_cfg['lock_joints']},execute_real=False))
            class ArmState:
                hz=30
                def read_joints(self):return q.copy()
                def execute(self,*a,**k):raise AssertionError('Planning process cannot execute')
            count=0
            for line in sys.stdin:
                request=json.loads(line)
                if request.get('op')=='close':break
                if request.get('op') not in ('plan','stage'):raise ValueError('Only planning is allowed')
                q=np.asarray(request['q'],dtype=float)
                if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Invalid simulated joints')
                observation=Observation.from_dict(request['observation'])
                if observation.source!='simulation':raise ValueError('Only explicit simulation observations accepted')
                feedback=request.get('simulated_gripper_joint_positions')
                if feedback is not None:
                    from rm75_app.planning.gripper_collision import gripper_link_transforms
                    expected=set(robot_cfg['lock_joints'])
                    if set(feedback)!=expected or any(not np.isfinite(v) or
                            abs(v-backend.config.gripper_collision_closed_joint_position)>.02 for v in feedback.values()):
                        raise ValueError('Invalid simulated closed-gripper feedback')
                    controller=backend._ensure_gripper_sphere_controller()
                    controller.state_transforms['closed']=gripper_link_transforms(robot_cfg['urdf_path'],feedback)
                    backend.set_gripper_collision_state(closed=True)
                proposals=[(Push(**raw),{}) for raw in request.get('push_candidates',[request.get('push')])]
                if not 1<=len(proposals)<=128:raise ValueError('Expected 1..128 ranked push candidates')
                events=[];tick=time.monotonic();periodic_audits.clear();motion_diagnostics.clear()
                class Events:
                    def emit(self,event,**values):
                        row=dict(event=event,**values);events.append(row)
                        if event=='physics_retreat_native_rejected':
                            # Preserve one bounded diagnostic even when candidate
                            # search is cancelled before plan_NNN.json is written.
                            atomic_json(args.directory/'last_retreat_rejection.json',
                                dict(row,source_observation=request['observation'],
                                     planning_request=count+1,execute_real=False))
                executor=AuditedPhysicsPushExecutor(backend,ArmState(),Config.from_dict({**data['model'],
                    'response_fits':request.get('response_fits',data['model'].get('response_fits',[]))}),data['motion'],
                    StopToken(args.directory/'STOP'),Events(),None)
                result=dict(execute_real=False,hardware_connected=False,hardware_profile_qualified=False,
                    ignore_gripper_internal_self_collision=backend.config.ignore_gripper_internal_self_collision,
                    retreat_collision_checks=False,complete_chain=False,validation_success=False,events=events,source_observation=request['observation'])
                try:
                    if request['op']=='stage':
                        from rm75_app.pusht.stage_planning import replan_stage
                        result.update(replan_stage(executor,observation,request))
                    elif 'push_candidates' in request:
                        selected,prepared=executor.plan_push_candidates(proposals,observation)
                    else:
                        selected=0;prepared=executor.plan_push(proposals[0][0],observation)
                    if request['op']=='plan':
                        result.update(complete_chain=True,validation_success=True,
                            contact_binding=executor.last_contact_binding,
                            retreat_native_audit_scope='predicted_post_push_ensemble',
                            retreat_post_push_scene_verified=False,
                            selected_candidate=selected,selected_push=proposals[selected][0].as_dict(),
                            stages=[dict(stage=s,positions=p.tolist(),times=t.tolist()) for s,p,t in prepared.stages])
                except Exception as exc:result['error']=f'{type(exc).__name__}: {exc}'
                result['response_fits']=list(executor.config.response_fits)
                result['periodic_ik_audit']=list(periodic_audits)
                result['motion_diagnostics']=list(motion_diagnostics)
                result['simulated_gripper_joint_positions']=feedback
                result['elapsed_s']=time.monotonic()-tick;count+=1
                path=args.directory/f'plan_{count:03d}.json';atomic_json(path,result)
                send(dict(result=str(path),success=result.get('stage_validated',result['complete_chain']),elapsed_s=result['elapsed_s']))
    except BaseException as exc:
        send(dict(ready=False,error=f'{type(exc).__name__}: {exc}'))
        raise
    finally:
        if backend is not None:backend.__exit__(None,None,None)


if __name__=='__main__':main()
