"""RM75/cuRobo2 motion bridge for the bounded planar pushes.

Before each action: plan the entire approach-contact-push-lift chain; audit TCP
corridors, collision contacts and all emitted joint samples. Tool mounting and
contact links come ONLY from the server-side qualified hardware profile.
"""
from __future__ import annotations
from dataclasses import replace
import time
import numpy as np
from rm75_app.workcell.io import finite
from rm75_app.workcell.transforms import vector, quaternion_matrix, rotation_error
from rm75_app.workcell.realman import time_parameterize
from .model import vertices, rectangles, predict, wrap


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
                raise RuntimeError(f'PushT collision audit rejected path: {forbidden[:3]}')

    def execute_push(self,push,obs):
        from rm75_app.planning.contracts import BatchPlanningRequest,JointConfiguration,Pose,PoseCandidate
        self.stop.check();scene=self._scene(obs);self.backend.update_scene(scene)
        native=self.backend._ensure_planner();self.names=tuple(native.joint_names)
        if self.names!=tuple(f'joint_{i}' for i in range(1,8)):
            raise ValueError('Unqualified RM75 joint order')
        q=self.arm.read_joints();start_q=q.copy()
        direction=np.asarray(push.direction);contact=np.asarray(push.contact)
        entry=contact-direction*self.config.approach_gap_m
        end=contact+direction*push.length_m
        points=[('approach',[*entry,self.z+self.hover],False,False),
                ('descend',[*entry,self.z],True,False),
                ('contact',[*contact,self.z],True,True),
                ('push',[*end,self.z],True,True),
                ('retreat',[*end,self.z+self.hover],True,True)]
        prepared=[]
        for stage,xyz,straight,contact_allowed in points:
            self.stop.check();pose=Pose(xyz,self.orientation)
            candidate=PoseCandidate(f'pusht:{stage}',pose)
            request=BatchPlanningRequest(JointConfiguration(self.names,q),(candidate,),scene,
                                         tool_frame=self.tool_frame,prefer_direct_tcp_path=straight)
            snapshots={name:self.backend._obstacle_enabled(name) for name in ('pusht_target_0','pusht_target_1')}
            try:
                if contact_allowed:
                    for name in snapshots:
                        self.backend._set_obstacle_enabled(name,False)
                planned=self.backend.plan_candidates(request).best((candidate,))
            finally:
                for name,enabled in snapshots.items():
                    self.backend._set_obstacle_enabled(name,enabled)
            if planned is None or planned.trajectory is None or len(planned.trajectory.positions)<2:
                raise RuntimeError(f'PushT motion planning failed at {stage}')
            path=planned.trajectory.positions
            if abs(path[0]-q).max()>.05:
                raise RuntimeError(f'Planner returned discontinuity at {stage}')
            start_pose=self._fk(q);delta=np.asarray(xyz)-start_pose.position
            for joints in path:
                actual=self._fk(joints)
                if straight:
                    t=np.clip(np.dot(actual.position-start_pose.position,delta)/max(np.dot(delta,delta),1e-12),0,1)
                    gap=np.linalg.norm(actual.position-(start_pose.position+t*delta))
                    if gap>self.corridor or rotation_error(quaternion_matrix(actual.quaternion_wxyz),quaternion_matrix(self.orientation))>self.orientation_tolerance:
                        raise RuntimeError(f'non_cartesian_contact_path:{stage}')
            final=self._fk(path[-1])
            if np.linalg.norm(final.position-xyz)>self.corridor:
                raise RuntimeError(f'endpoint_error:{stage}')
            timed,ts=time_parameterize(path,lambda joints:self._fk(joints).position,
                 speed_mps=push.speed_mps,hz=self.arm.hz,
                 joint_speed_rad_s=self.profile.get('joint_speed_rad_s',.25),
                 joint_accel_rad_s2=self.profile.get('joint_accel_rad_s2',.5))
            self._audit(timed,contact=contact_allowed)
            if stage in ('push','retreat'):
                # Conservative sampled swept-target ensemble: the pusher may
                # contact the T, but the palm/other links may not intersect its
                # possible translated/rotated geometry. Not continuous proof.
                try:
                    for scale in self.config.friction_scales:
                        for fraction in (.5,1.):
                            future=predict(obs.pose,replace(push,length_m=push.length_m*fraction),self.config,scale)
                            self.backend.update_scene(self._scene(replace(obs,pose=tuple(future))))
                            self._audit(timed,contact=True)
                finally:
                    self.backend.update_scene(scene)
            prepared.append((stage,timed,ts));q=path[-1]
        # Planning may take longer than the image freshness window. Obtain a NEW
        # measurement and reject drift; never just renew the old timestamp.
        current=self.observer.observe(after=obs.captured_at)
        current.validate(previous=obs,max_age_s=self.config.max_observation_age_s,real=True)
        if np.linalg.norm(np.asarray(current.pose)[:2]-np.asarray(obs.pose)[:2])>.003 or abs(wrap(current.pose[2]-obs.pose[2]))>.04:
            raise RuntimeError('Object moved while planning; re-observe/replan required')
        if abs(self.arm.read_joints()-start_q).max()>self.arm.start_gap:
            raise RuntimeError('Robot moved while planning')
        self.events.emit('push_chain_audited',stages=[s for s,_,_ in prepared],
                         samples=sum(len(p) for _,p,_ in prepared),speed_mps=push.speed_mps)
        for stage,path,ts in prepared:
            self.stop.check()
            self.arm.execute(path,ts,stage=stage)
