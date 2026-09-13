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
        self.construction=request.get('native_tool_construction')

    def sample(self,t):
        i=max(0,min(int(np.searchsorted(self.times,t,side='right')-1),len(self.times)-2))
        u=(t-self.times[i])/(self.times[i+1]-self.times[i])
        return {name:interpolate_pose(poses[i],poses[i+1],u) for name,poses in self.poses.items()}

    def build(self, scene, spose):
        import sapien
        import hashlib
        from pathlib import Path
        from types import SimpleNamespace
        from .native_construction import NativeConstructionRecipe
        from .native_body_mirror import shape_state,compare_native_state
        if not self.construction:raise ValueError('Original native tool construction adapter required')
        path=Path(self.construction['urdf'])
        if hashlib.sha256(path.read_bytes()).hexdigest()!=self.construction['urdf_sha256']:
            raise ValueError('Original native tool URDF changed')
        loader=scene.create_urdf_loader();loader.fix_root_link=True
        loader.disable_self_collisions=False;loader.load_multiple_collisions_from_file=True
        parsed=loader.parse(str(path))
        if len(parsed['articulation_builders'])!=1 or parsed['actor_builders']:
            raise ValueError('Original single robot articulation required')
        source={link.name:link for link in parsed['articulation_builders'][0].link_builders}
        bodies={};entities=[]
        for name,states in self.links.items():
            entity=sapien.Entity();entity.name='measured_'+name
            builder=scene.create_actor_builder();builder.physx_body_type='kinematic'
            builder.collision_records=list(source[name].collision_records)
            builder.collision_groups=list(source[name].collision_groups)
            recipe=NativeConstructionRecipe(builder,SimpleNamespace(_objs=()))
            body=recipe.build_body(np.eye(4));body.kinematic=True
            if len(body.collision_shapes)!=len(states):
                raise ValueError('Original native loader changed tool shape count')
            for shape,state in zip(body.collision_shapes,states):
                shape.physical_material=sapien.physx.PhysxMaterial(**state['material'])
                shape.set_collision_groups(state['collision_groups'])
                for key,value in state['properties'].items():setattr(shape,key,value)
                compare_native_state(state,shape_state(shape,np.eye(4)),name)
            entity.add_component(body);entity.set_pose(spose(self.poses[name][0]))
            scene.sub_scenes[0].add_entity(entity)
            bodies[name]=body;entities.append(entity)
        return bodies,entities
