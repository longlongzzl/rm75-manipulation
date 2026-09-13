"""Replay recorded native tool links, never commanded jaw or arm positions."""
import numpy as np
from .scene import digest, transform, pose_error
from .physics_replay import interpolate_pose


class NativeToolProgram:
    def __init__(self, request, geometry, motion):
        canonical=dict(motion); claimed=canonical.pop('motion_digest')
        if digest(canonical)!=claimed or motion['geometry_digest']!=digest(geometry):
            raise ValueError('Native tool evidence digest mismatch')
        if (geometry.get('schema')!='rm75_native_tool_geometry_v1' or
                motion.get('schema')!='rm75_native_tool_motion_v1' or
                motion.get('source')!='native_link_pose_readback'):
            raise ValueError('Native tool readback evidence required')
        for key in ('action_digest','transition_digest'):
            if motion[key]!=request[key]:raise ValueError('Native tool action binding changed')
        action=request['actual_action']; rows=motion['samples']
        self.links=geometry['links']; self.times=np.asarray(action['time_s'],float)
        if not 2<=len(rows)<=100000 or len(rows)!=len(self.times) or np.any(np.diff(self.times)<=0):
            raise ValueError('Native tool motion coverage invalid')
        self.poses={name:[] for name in self.links}
        for row,t,tcp in zip(rows,self.times,action['T_world_tcp']):
            p,r=pose_error(row['T_world_tcp'],tcp)
            if abs(row['time_s']-t)>1e-9 or p>1e-9 or r>1e-7 or set(row['link_poses'])!=set(self.links):
                raise ValueError('Native tool motion differs from actual action')
            for name in self.links:self.poses[name].append(transform(row['link_poses'][name]))
        self.geometry_digest=motion['geometry_digest'];self.motion_digest=claimed

    def sample(self,t):
        i=max(0,min(int(np.searchsorted(self.times,t,side='right')-1),len(self.times)-2))
        u=(t-self.times[i])/(self.times[i+1]-self.times[i])
        return {name:interpolate_pose(poses[i],poses[i+1],u) for name,poses in self.poses.items()}

    def build(self, scene, spose):
        import sapien
        from .native_body_mirror import shape_state,compare_native_state
        bodies={};entities=[]
        for name,states in self.links.items():
            entity=sapien.Entity();entity.name='measured_'+name
            body=sapien.physx.PhysxRigidDynamicComponent();body.kinematic=True
            for state in states:
                if state['kind']!='ConvexMesh':raise ValueError('Native tool shape adapter missing')
                g=state['geometry']
                shape=sapien.physx.PhysxCollisionShapeConvexMesh(
                    np.asarray(g['vertices'],dtype=np.float32),np.asarray(g['scale'],dtype=np.float32),
                    sapien.physx.PhysxMaterial(**state['material']))
                shape.local_pose=spose(state['T_object_shape'])
                shape.set_collision_groups(state['collision_groups'])
                for key,value in state['properties'].items():setattr(shape,key,value)
                compare_native_state(state,shape_state(shape,np.eye(4)),name)
                body.attach(shape)
            entity.add_component(body);entity.set_pose(spose(self.poses[name][0]))
            scene.sub_scenes[0].add_entity(entity)
            bodies[name]=body;entities.append(entity)
        return bodies,entities
