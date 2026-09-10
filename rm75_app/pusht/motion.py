"""RM75/cuRobo2 motion bridge for the bounded planar pushes.

Before each action: plan the entire approach-contact-push-lift chain; audit TCP
corridors and all emitted joint samples. Collision checks exempt only the
user-requested retreat backoff/lift. Tool mounting and
contact links come ONLY from the server-side qualified hardware profile.
"""
from __future__ import annotations
from dataclasses import dataclass, replace
import time
import numpy as np
from rm75_app.workcell.io import finite
from rm75_app.workcell.transforms import vector, quaternion_matrix, rotation_error
from rm75_app.workcell.realman import time_parameterize
from .model import vertices, rectangles, predict, wrap
from .cartesian_ik import plan_cartesian_line,PushPathRejected


def pusht_planner_options(options=None):
    """PushT uses the user's gripper-internal self-collision exclusion."""
    result = dict(options or {})
    result.setdefault('ignore_gripper_internal_self_collision', True)
    return result


@dataclass(frozen=True)
class PreparedPush:
    """A complete audited chain, not an authorization to execute it."""
    stages: tuple
    start_q: np.ndarray


class CuroboPushExecutor:
    def __init__(self,backend,arm,config,profile,stop,events,observer):
        self.backend=backend;self.arm=arm;self.config=config;self.profile=profile
        self.stop=stop;self.events=events;self.observer=observer
        self.tool_frame=str(profile['tool_frame'])
        self.z=finite(profile['push_tcp_z_m'],'push_tcp_z_m',-1,2)
        self.hover=finite(profile['hover_clearance_m'],'hover_clearance_m',.02,.2)
        self.orientation=vector(profile['tool_quaternion_wxyz'],4,'tool_quaternion')
        self.allowed=set(profile['pusher_contact_links'])
        if not self.allowed or profile.get('tool_collision_geometry_verified') is not True:
            raise PermissionError('Verified pusher collision geometry/contact links are required')
        self.corridor=finite(profile.get('corridor_tolerance_m',.003),'corridor_tolerance_m',.0005,.005)
        self.orientation_tolerance=finite(profile.get('orientation_tolerance_rad',.05),'orientation_tolerance_rad',.005,.1)
        if self.tool_frame != 'gripper_tcp':
            raise PermissionError('Closed gripper mapping requires the original gripper_tcp frame')

    def _scene(self,obs):
        from rm75_app.planning.contracts import PlanningScene,CollisionObject,Pose
        from .model import rotation
        p=np.asarray(obs.pose);r=rotation(p[2]);objects=[]
        yawq=[np.cos(p[2]/2),0,0,np.sin(p[2]/2)]
        for index,(x,y,w,h) in enumerate(rectangles(self.config)):
            xy=p[:2]+r@[x,y]
            objects.append(CollisionObject(f'pusht_target_{index}','cuboid',
                Pose([*xy,self.profile['object_centroid_z_m']],yawq),
                dimensions=[w,h,self.profile['object_height_m']]))
        for raw in self.profile['static_collision_objects']:
            objects.append(CollisionObject(raw['name'],raw['kind'],Pose(raw['position'],raw['quaternion_wxyz']),
                                           **{k:raw[k] for k in ('dimensions','radius','mesh_path','scale') if k in raw}))
        if len(objects)<3:
            raise ValueError('PushT scene must include a calibrated table collision object')
        return PlanningScene(tuple(objects),revision=f'{obs.session_id}:{obs.sequence}')

    def _fk(self,q):
        from rm75_app.planning.contracts import JointConfiguration
        return self.backend.tool_pose_for_configuration(JointConfiguration(self.names,q),self.tool_frame)

    def _audit(self,path,*,contact):
        backend=self.backend;planner=backend._ensure_planner();mods=backend._import_modules()
        # Chunked exhaustive sampled-path audit, NOT five-point subsampling.
        for offset in range(0,len(path),32):
            self.stop.check()
            state=mods['JointState'].from_position(
                mods['torch'].as_tensor(path[offset:offset+32],device=planner.device_cfg.device,dtype=planner.device_cfg.dtype),
                joint_names=list(self.names))
            contacts=backend._collision_diagnostics_for_states(planner,{'path':state})
            forbidden=[c for c in contacts if not (contact and c.get('collision_type')=='world'
                         and c.get('world_object') in ('pusht_target_0','pusht_target_1')
                         and c.get('robot_link') in self.allowed)]
            if forbidden:
                raise PushPathRejected(f'PushT collision audit rejected path: {forbidden[:3]}')

    def _audit_tcp(self,path,start_pose,xyz,*,straight,stage,check_endpoint=True):
        delta=np.asarray(xyz)-start_pose.position
        worst_gap=0.;worst_angle=0.
        for joints in path:
            actual=self._fk(joints)
            angle=rotation_error(quaternion_matrix(actual.quaternion_wxyz),quaternion_matrix(self.orientation))
            if straight:
                t=np.clip(np.dot(actual.position-start_pose.position,delta)/max(np.dot(delta,delta),1e-12),0,1)
                gap=float(np.linalg.norm(actual.position-(start_pose.position+t*delta)))
                worst_gap=max(worst_gap,gap);worst_angle=max(worst_angle,angle)
        if straight and (worst_gap>self.corridor or worst_angle>self.orientation_tolerance):
            raise PushPathRejected(f'non_cartesian_contact_path:{stage}: max_gap_m={worst_gap:.8f}, max_angle_rad={worst_angle:.8f}')
        final=self._fk(path[-1])
        if check_endpoint and (np.linalg.norm(final.position-xyz)>self.corridor or
            rotation_error(quaternion_matrix(final.quaternion_wxyz),quaternion_matrix(self.orientation))>self.orientation_tolerance):
            raise PushPathRejected(f'endpoint_error:{stage}')
        return dict(max_corridor_error_m=worst_gap if straight else None,
                    max_orientation_error_rad=worst_angle if straight else None)

    def _plan_retreat(self, q, push, scene, binding):
        """Back off opposite the push, then lift in world Z without collision gates."""
        from rm75_app.planning.contracts import BatchPlanningRequest,JointConfiguration,Pose,PoseCandidate
        parts=[];clock=0.;segments=[]
        with self.backend.suspend_collision_checks():
            for label,xyz in (('backoff', binding['tcp_retreat_backoff_xyz']),
                              ('lift', binding['tcp_retreat_lift_xyz'])):
                self.stop.check();xyz=np.asarray(xyz);start_pose=self._fk(q)
                stage=f'retreat:{label}'
                candidate=PoseCandidate(f'pusht:{stage}',Pose(xyz,self.orientation))
                request=BatchPlanningRequest(JointConfiguration(self.names,q),(candidate,),scene,
                                            tool_frame=self.tool_frame,prefer_direct_tcp_path=True)
                def validate(edge):
                    self.stop.check()
                    self._audit_tcp(edge,start_pose,xyz,straight=True,stage=stage,check_endpoint=False)
                path,ik_rows=plan_cartesian_line(self.backend,request,validate)
                self._audit_tcp(path,start_pose,xyz,straight=True,stage=stage)
                timed,ts=time_parameterize(path,lambda joints:self._fk(joints).position,
                    speed_mps=push.speed_mps,hz=self.arm.hz,
                    joint_speed_rad_s=self.profile.get('joint_speed_rad_s',.25),
                    joint_accel_rad_s2=self.profile.get('joint_accel_rad_s2',.5))
                metrics=self._audit_tcp(timed,start_pose,xyz,straight=True,stage=stage)
                if len(timed)<2 or abs(timed[0]-q).max()>1e-5:
                    raise PushPathRejected(f'Invalid retreat segment continuity: {label}')
                self.events.emit('push_cartesian_ik_planned',stage=stage,waypoints=ik_rows,
                                 collision_checks=False)
                segments.append(dict(segment=label,start_xyz=start_pose.position.tolist(),
                    goal_xyz=xyz.tolist(),samples=len(timed),duration_s=float(ts[-1]),**metrics))
                parts.append((timed if not parts else timed[1:], (ts+clock) if not parts else (ts+clock)[1:]))
                clock+=float(ts[-1]);q=timed[-1]
        path=np.concatenate([p for p,_ in parts]);times=np.concatenate([t for _,t in parts])
        self.events.emit('push_stage_audited',stage='retreat',samples=len(path),duration_s=clock,
                         collision_checks=False,collision_policy='user_requested_backoff_lift_exemption',
                         segments=segments)
        return path,times

    def contact_direction_mask(self,obs,config):
        from .closed_gripper import contact_direction_mask
        if obs.source != 'simulation' and self.profile.get('closed_gripper_verified') is not True:
            raise PermissionError('Actual closed gripper state must be verified before hardware planning')
        self.backend.set_gripper_collision_state(closed=True)
        geometry=self.backend.closed_gripper_tool_geometry(self.arm.read_joints())
        return contact_direction_mask(obs,config,self.profile,geometry,self._scene(obs))

    def plan_push_candidates(self, proposals, obs):
        """Compare ranked actions with batches of native approach trajectories."""
        from rm75_app.planning.contracts import BatchPlanningRequest,JointConfiguration,Pose,PoseCandidate
        from .closed_gripper import bind_push
        if obs.source != 'simulation' and self.profile.get('closed_gripper_verified') is not True:
            raise PermissionError('Actual closed gripper state must be verified before hardware planning')
        scene=self._scene(obs);self.backend.update_scene(scene)
        native=self.backend._ensure_planner();self.names=tuple(native.joint_names)
        q=self.arm.read_joints();self.backend.set_gripper_collision_state(closed=True)
        geometry=self.backend.closed_gripper_tool_geometry(q)
        candidates=[];rejections=[]
        for index,(push,_) in enumerate(proposals):
            self.stop.check()
            try:
                points,_=bind_push(push,obs,self.config,self.profile,geometry,scene)
            except PushPathRejected as exc:
                rejections.append(dict(rank=index,phase='tool_geometry',error=str(exc)))
                continue
            candidates.append((index,push,PoseCandidate(f'pusht:approach:candidate_{index}',
                                                       Pose(points[0][1],self.orientation))))
        batch_size=min(4,self.backend.config.max_batch_size)
        for offset in range(0,len(candidates),batch_size):
            chunk=candidates[offset:offset+batch_size];self.stop.check();self.backend.update_scene(scene)
            request=BatchPlanningRequest(JointConfiguration(self.names,q),tuple(row[2] for row in chunk),
                                         scene,tool_frame=self.tool_frame)
            result=self.backend.plan_candidates(request)
            self.events.emit('push_candidate_batch_planned',ranks=[row[0] for row in chunk],
                             native_batch_size=len(chunk))
            for index,push,candidate in chunk:
                plan=next((p for p in result.plans if p.candidate_id==candidate.candidate_id and p.success),None)
                if plan is None or plan.trajectory is None:
                    rejections.append(dict(rank=index,phase='approach',error='trajectory_failed'))
                    continue
                try:
                    prepared=self.plan_push(push,obs,_approach_path=plan.trajectory.positions)
                except (PushPathRejected,RuntimeError) as exc:
                    if not isinstance(exc,PushPathRejected) and not str(exc).startswith('PushT motion planning failed'):
                        raise
                    rejections.append(dict(rank=index,phase='complete_chain',error=str(exc)))
                    continue
                self.events.emit('push_candidate_selected',rank=index,proposals=len(proposals),
                                 geometry_valid=len(candidates),rejections=rejections)
                return index,prepared
        raise PushPathRejected(f'no_executable_push_candidate: {rejections}')

    def prepare_push_candidates(self,proposals,obs,*,model=None):
        if model is not None:self.config=model
        index,prepared=self.plan_push_candidates(proposals,obs)
        self._prepared_selection=(proposals[index][0],obs.as_dict(),prepared)
        return index

    def plan_push(self,push,obs,*,_approach_path=None):
        """Plan and audit all five stages without issuing any arm command.

        Works with a read-only joint-state provider for GPU no-motion gates;
        it does not connect hardware or mark any profile qualified.
        """
        from rm75_app.planning.contracts import BatchPlanningRequest,JointConfiguration,Pose,PoseCandidate
        self.stop.check();scene=self._scene(obs);self.backend.update_scene(scene)
        native=self.backend._ensure_planner();self.names=tuple(native.joint_names)
        if self.names!=tuple(f'joint_{i}' for i in range(1,8)):
            raise ValueError('Unqualified RM75 joint order')
        q=self.arm.read_joints();start_q=q.copy()
        if obs.source != 'simulation' and self.profile.get('closed_gripper_verified') is not True:
            raise PermissionError('Actual closed gripper state must be verified before hardware planning')
        self.backend.set_gripper_collision_state(closed=True)  # Model only, not a gripper command.
        from .closed_gripper import bind_push
        geometry=self.backend.closed_gripper_tool_geometry(q)
        points,binding=bind_push(push,obs,self.config,self.profile,geometry,scene)
        self.events.emit('push_closed_gripper_binding',**binding)
        self.last_contact_binding=binding
        prepared=[]
        for stage,xyz,straight,contact_allowed in points:
            self.stop.check()
            if stage == 'retreat':
                timed,ts=self._plan_retreat(q,push,scene,binding)
                prepared.append((stage,timed,ts));q=timed[-1]
                continue
            pose=Pose(xyz,self.orientation)
            start_pose=self._fk(q)
            candidate=PoseCandidate(f'pusht:{stage}',pose)
            request=BatchPlanningRequest(JointConfiguration(self.names,q),(candidate,),scene,
                                         tool_frame=self.tool_frame,prefer_direct_tcp_path=straight)
            snapshots={name:self.backend._obstacle_enabled(name) for name in ('pusht_target_0','pusht_target_1')}
            try:
                if contact_allowed:
                    for name in snapshots:
                        self.backend._set_obstacle_enabled(name,False)
                if straight:
                    def validate_edge(edge):
                        self._audit_tcp(edge,start_pose,xyz,straight=True,stage=stage,check_endpoint=False)
                        self._audit(edge,contact=contact_allowed)
                    path,ik_rows=plan_cartesian_line(self.backend,request,validate_edge)
                    self.events.emit('push_cartesian_ik_planned',stage=stage,waypoints=ik_rows)
                elif stage == 'approach' and _approach_path is not None:
                    path=_approach_path
                else:
                    planned=self.backend.plan_candidates(request).best((candidate,))
                    if planned is None or planned.trajectory is None:
                        raise RuntimeError(f'PushT motion planning failed at {stage}')
                    path=planned.trajectory.positions
            finally:
                for name,enabled in snapshots.items():
                    self.backend._set_obstacle_enabled(name,enabled)
            if len(path)<2:
                raise RuntimeError(f'PushT motion planning failed at {stage}')
            if abs(path[0]-q).max()>.05:
                raise RuntimeError(f'Planner returned discontinuity at {stage}')
            self._audit_tcp(path,start_pose,xyz,straight=straight,stage=stage)
            timed,ts=time_parameterize(path,lambda joints:self._fk(joints).position,
                 speed_mps=push.speed_mps,hz=self.arm.hz,
                 joint_speed_rad_s=self.profile.get('joint_speed_rad_s',.25),
                 joint_accel_rad_s2=self.profile.get('joint_accel_rad_s2',.5))
            metrics=self._audit_tcp(timed,start_pose,xyz,straight=straight,stage=stage)
            self._audit(timed,contact=contact_allowed)
            if stage == 'push':
                # Conservative sampled swept-target ensemble: the pusher may
                # contact the T, but the palm/other links may not intersect its
                # possible translated/rotated geometry. Not continuous proof.
                try:
                    for scale in self.config.friction_scales:
                        for fraction in (.5,1.):
                            # Force acts at the bound object surface, never the surrogate circle/TCP center.
                            effective=replace(push,contact=tuple(binding['surface_contact_xyz'][:2]),
                                              length_m=push.length_m*fraction)
                            future=predict(obs.pose,effective,self.config,scale)
                            self.backend.update_scene(self._scene(replace(obs,pose=tuple(future))))
                            self._audit(timed,contact=True)
                finally:
                    self.backend.update_scene(scene)
            self.events.emit('push_stage_audited',stage=stage,samples=len(timed),
                             duration_s=float(ts[-1]),collision_checks=True,**metrics)
            prepared.append((stage,timed,ts));q=path[-1]
        self.events.emit('push_chain_planned',stages=[s for s,_,_ in prepared],
                         samples=sum(len(p) for _,p,_ in prepared),speed_mps=push.speed_mps)
        return PreparedPush(tuple(prepared),start_q)

    def execute_push(self,push,obs):
        # Observation labels never grant permission to assume a real jaw state.
        if self.profile.get('closed_gripper_verified') is not True:
            raise PermissionError('Actual closed gripper state must be verified before execution')
        cached=getattr(self,'_prepared_selection',None);self._prepared_selection=None
        if cached is not None and cached[0]==push and cached[1]==obs.as_dict():
            planned=cached[2]
        else:
            planned=self.plan_push(push,obs)
        prepared=planned.stages;start_q=planned.start_q
        # Planning may take longer than the image freshness window. Obtain a NEW
        # measurement and reject drift; never just renew the old timestamp.
        current=self.observer.observe(after=obs.captured_at)
        current.validate(previous=obs,after=obs.captured_at,max_age_s=self.config.max_observation_age_s,real=True)
        if np.linalg.norm(np.asarray(current.pose)[:2]-np.asarray(obs.pose)[:2])>.003 or abs(wrap(current.pose[2]-obs.pose[2]))>.04:
            raise RuntimeError('Object moved while planning; re-observe/replan required')
        if abs(self.arm.read_joints()-start_q).max()>self.arm.start_gap:
            raise RuntimeError('Robot moved while planning')
        self.events.emit('push_chain_audited',stages=[s for s,_,_ in prepared],
                         samples=sum(len(p) for _,p,_ in prepared),speed_mps=push.speed_mps)
        for stage,path,ts in prepared:
            self.stop.check()
            self.arm.execute(path,ts,stage=stage)
