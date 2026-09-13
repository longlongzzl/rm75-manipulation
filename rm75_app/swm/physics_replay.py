"""Actual PhysX replay adapter for SWM friction/density hypotheses.

Uses the existing ManiSkill/SAPIEN contact-environment infrastructure, but builds
an independent world from the measured SWM. The tool follows MEASURED TCP poses.
The dynamic target is stepped by physics; it is never assigned predicted poses.
This is system identification, not a full-arm collision/servo qualification.
No GPU, camera, robot SDK or simulator is imported until the isolated worker runs.
"""
from __future__ import annotations
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import numpy as np
from .scene import transform, digest
from .identification import PhysicsParameters


def interpolate_pose(a, b, ratio):
    # scipy is already part of the physics/pose environments; lazy import keeps
    # the SWM state/tool modules independent of simulator dependencies.
    from scipy.spatial.transform import Rotation, Slerp
    a,b=transform(a),transform(b);u=float(np.clip(ratio,0,1))
    out=np.eye(4);out[:3,3]=(1-u)*a[:3,3]+u*b[:3,3]
    rotations=Rotation.from_matrix(np.stack([a[:3,:3],b[:3,:3]]))
    out[:3,:3]=Slerp([0,1],rotations)([u]).as_matrix()[0]
    return out


class MeasuredTCPProgram:
    def __init__(self, action):
        self.times=np.asarray(action['time_s'],dtype=float)
        self.poses=[transform(p) for p in action['T_world_tcp']]
        self.stages=action.get('stages')
        if not isinstance(self.stages,list) or len(self.stages)!=len(self.times) or any(s not in ('approach','descend','contact','push','retreat','post_settle') for s in self.stages):
            raise ValueError('Replay requires the measured action phase for every TCP sample')
        self.duration=float(self.times[-1]);self.initial=np.zeros(7)
    def fk(self, q):return self.poses[0].copy()  # only used by inherited tool initialization
    def sample(self,t):
        index=max(0,min(int(np.searchsorted(self.times,t,side='right')-1),len(self.times)-2))
        ratio=(t-self.times[index])/(self.times[index+1]-self.times[index])
        return self.stages[index],interpolate_pose(self.poses[index],self.poses[index+1],ratio)


class SubprocessReplayWorld:
    """One isolated OS process per physical hypothesis, with a wall timeout."""
    domain='physics'
    def __init__(self, request, *, python, tool_spheres=None, directory, timeout_s=180, check=lambda:None,
                 native_tool_geometry=None, native_tool_motion=None):
        if not Path(python).is_file():raise FileNotFoundError('Configure the existing physics Python interpreter')
        if not 1<=timeout_s<=900:raise ValueError('Replay timeout must be bounded')
        self.request=copy.deepcopy(request);self.python=str(python);self.timeout=timeout_s
        self.check=check;self.process=None;self.directory=Path(directory)/request['hypothesis_id']
        if self.directory.exists():raise FileExistsError('Use a new transition evidence directory')
        self.directory.mkdir(parents=True)
        if native_tool_geometry is not None or native_tool_motion is not None:
            from .native_tool_replay import NativeToolProgram
            NativeToolProgram(request,native_tool_geometry,native_tool_motion)
            self.request['native_tool_geometry']=copy.deepcopy(native_tool_geometry)
            self.request['native_tool_motion']=copy.deepcopy(native_tool_motion)
            self.request['tool_spheres']=[]
            return
        spheres=np.asarray(tool_spheres,dtype=float)
        if spheres.ndim!=2 or spheres.shape[1]!=4 or not len(spheres) or not np.isfinite(spheres).all() or (spheres[:,3]<=0).any():
            raise ValueError('Calibrated tool collision spheres required')
        self.request['tool_spheres']=spheres.tolist()
    def replay(self):
        root=Path(__file__).resolve().parents[2]
        request_path=self.directory/'request.json';result_path=self.directory/'result.json'
        request_path.write_text(json.dumps(self.request,allow_nan=False),encoding='utf-8')
        command=[self.python,str(root/'tools/run_network_isolated.py'),'--',self.python,
                 '-m','rm75_app.swm.physics_replay','--request',str(request_path),'--result',str(result_path)]
        env=dict(os.environ);env['PYTHONPATH']=str(root)+os.pathsep+env.get('PYTHONPATH','')
        with (self.directory/'worker.log').open('wb') as log:
            self.process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,
                                          stdin=subprocess.DEVNULL,start_new_session=True)
            deadline=time.monotonic()+self.timeout
            while self.process.poll() is None:
                self.check()
                if time.monotonic()>deadline:raise TimeoutError('Physical hypothesis replay timed out')
                time.sleep(.05)
        if self.process.returncode!=0 or not result_path.is_file():raise RuntimeError('Physical replay worker failed')
        if result_path.stat().st_size>16000000:raise ValueError('Replay output exceeds evidence budget')
        return json.loads(result_path.read_text())
    def close(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGTERM)
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=2)


def require_physical_replay_assets(assets):
    for asset in assets.values():
        if asset.get('collision_role') == 'planning_proxy':
            raise ValueError('Native physical geometry replay adapter required; planning proxies are not physical models')
        if asset.get('collision_role') not in (None, 'physical'):
            raise ValueError('Unknown physical replay asset role')


def compound_cuboid_parts(asset):
    """Read metric parts from the hashed physical descriptor, never a convex hull."""
    import hashlib
    path=Path(asset['collision_path'])
    if path.stat().st_size>1000000:raise ValueError('Compound collision descriptor too large')
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=asset['collision_sha256']:
        raise ValueError('Changed compound collision descriptor')
    value=json.loads(raw)
    if value.get('schema')!='rm75_compound_cuboids_v1' or value.get('units')!='m':
        raise ValueError('Metric compound collision descriptor required')
    parts=value.get('parts')
    if not isinstance(parts,list) or not 1<=len(parts)<=64:
        raise ValueError('Compound collision requires 1..64 parts')
    result=[]
    for part in parts:
        dims=np.asarray(part['dimensions_m'],dtype=float)
        if dims.shape!=(3,) or not np.isfinite(dims).all() or (dims<=0).any():
            raise ValueError('Invalid compound collision dimensions')
        result.append((transform(part['T_collision_part']),dims))
    return result


def physical_replay(request):
    """Native adapter. Imported/stepped only in the network-isolated subprocess."""
    require_physical_replay_assets(request['initial_snapshot']['assets'])
    import hashlib
    import sapien
    from mani_skill.utils.structs.types import SimConfig
    from rm75_app.simulation.pusht_contact_env import PushTContactEnv
    from types import SimpleNamespace
    from scipy.spatial.transform import Rotation
    theta=PhysicsParameters(**request['parameters']);snapshot=request['initial_snapshot']
    program=MeasuredTCPProgram(request['actual_action']);target_id=request['object_id']
    native_program=None
    if 'native_tool_geometry' in request:
        from .native_tool_replay import NativeToolProgram
        native_program=NativeToolProgram(request,request['native_tool_geometry'],request['native_tool_motion'])
    if request['action_digest']!=digest(request['actual_action']):raise ValueError('Actual action was modified')
    canonical=dict(snapshot);sid=canonical.pop('snapshot_id')
    if digest(canonical)!=sid:raise ValueError('Initial SWM snapshot was modified')
    sample_times=np.asarray(request['sample_times'],dtype=float)
    if sample_times[0]!=0 or sample_times[-1]>program.duration or np.any(np.diff(sample_times)<=0):
        raise ValueError('Invalid observation times for physical replay')
    for asset in snapshot['assets'].values():
        p=Path(asset['collision_path'])
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=asset['collision_sha256']:
            raise ValueError('Changed collision mesh')
        if asset.get('collision_kind') not in ('convex_mesh','cuboid','sphere','compound_cuboids'):
            raise ValueError('Replay requires a prevalidated convex collision proxy, not gaussian splats')
    def spose(matrix):
        a=transform(matrix);xyzw=Rotation.from_matrix(a[:3,:3]).as_quat()
        return sapien.Pose(a[:3,3],xyzw[[3,0,1,2]])
    class ReplayEnv(PushTContactEnv):
        @property
        def _default_sim_config(self):return SimConfig(sim_freq=240,control_freq=240)
        @property
        def _default_human_render_camera_configs(self):return []
        def _load_scene(self,options):
            material=sapien.physx.PhysxMaterial(theta.static_friction,theta.dynamic_friction,0.)
            self.world=[];self.dynamic_others=[]
            for oid,obj in snapshot['objects'].items():
                asset=snapshot['assets'][obj['asset_id']];builder=self.scene.create_actor_builder()
                builder.initial_pose=spose(obj['measured']['T_world_object'])
                density=(theta.density_kg_m3 if oid==target_id else asset.get('nominal_density_kg_m3'))
                if obj['fixed']:density=1000.  # static actor has no simulated mass
                if density is None or not np.isfinite(density) or density<=0:
                    raise ValueError('Non-target dynamic objects require explicit nominal density')
                local_pose=spose(asset.get('T_object_collision',np.eye(4)))
                if asset['collision_kind']=='convex_mesh':
                    builder.add_convex_collision_from_file(asset['collision_path'],pose=local_pose,material=material,density=density)
                elif asset['collision_kind']=='sphere':
                    radius=float(asset['collision_radius_m'])
                    if not np.isfinite(radius) or radius<=0:raise ValueError('Invalid collision sphere')
                    builder.add_sphere_collision(pose=local_pose,radius=radius,material=material,density=density)
                elif asset['collision_kind']=='compound_cuboids':
                    base=transform(asset.get('T_object_collision',np.eye(4)))
                    for part,dims in compound_cuboid_parts(asset):
                        builder.add_box_collision(pose=spose(base@part),half_size=dims/2,
                                                  material=material,density=density)
                else:
                    dims=np.asarray(asset['collision_dimensions_m'],dtype=float)
                    if dims.shape!=(3,) or not np.isfinite(dims).all() or (dims<=0).any():raise ValueError('Invalid collision box')
                    builder.add_box_collision(pose=local_pose,half_size=dims/2,material=material,density=density)
                actor=builder.build_static(name=oid) if obj['fixed'] else builder.build(name=oid)
                if oid==target_id:
                    if obj['fixed']:raise ValueError('Target may not be static')
                    self.target=actor;self.target_pose=spose(obj['measured']['T_world_object'])
                elif obj['fixed']:self.world.append(actor)
                else:self.dynamic_others.append(actor)
            self._load_tool(material,None)
        def _load_tool(self, material, tool_visual):
            if native_program is not None:
                self.native_bodies,self.native_entities=native_program.build(self.scene,spose)
                # A collision-free TCP marker supplies actual simulator pose readback.
            builder=self.scene.create_actor_builder();builder.initial_pose=spose(program.poses[0])
            for x,y,z,radius in request['tool_spheres']:
                builder.add_sphere_collision(pose=sapien.Pose([x,y,z]),radius=radius,material=material)
            self.tool=builder.build_kinematic(name='measured_actual_tool_replay')
            self.tool_body=self.tool._objs[0].find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        def _before_simulation_step(self):
            super()._before_simulation_step()
            if native_program is not None:
                for name,pose in native_program.sample(self.physics_time).items():
                    self.native_bodies[name].set_kinematic_target(spose(pose))
        def _after_simulation_step(self):
            if native_program is None:super()._after_simulation_step()
        def _initialize_episode(self,env_idx,options):
            super()._initialize_episode(env_idx,options);self.settle_s=0.
    env=None
    try:
        env=ReplayEnv(program=program,spheres=request['tool_spheres'],config=None,
            observation=SimpleNamespace(pose=[0,0,0]),motion={},static_objects=[],
            obs_mode='none',reward_mode='none',render_mode=None,sim_backend='physx_cpu',num_envs=1)
        env.reset(seed=0)
        def read():return env.target.pose.to_transformation_matrix().detach().cpu().numpy().reshape(4,4)
        previous=read();poses=[previous.tolist()];cursor=1;previous_t=0.
        def read_tool():return env.tool.pose.to_transformation_matrix().detach().cpu().numpy().reshape(4,4).tolist()
        feedback_times=[0.];feedback_poses=[read_tool()];feedback_stages=[program.stages[0]]
        while cursor<len(sample_times):
            env.step(None);now=float(env.physics_time);current=read()
            feedback_times.append(now);feedback_poses.append(read_tool())
            feedback_stages.append(program.sample(min(now,program.duration))[0])
            while cursor<len(sample_times) and sample_times[cursor]<=now:
                fraction=(sample_times[cursor]-previous_t)/(now-previous_t)
                poses.append(interpolate_pose(previous,current,fraction).tolist());cursor+=1
            previous,previous_t=current,now
        target_body=env.target._objs[0].find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        return dict(hypothesis_id=request['hypothesis_id'],parameters=request['parameters'],
                    action_digest=request['action_digest'],transition_digest=request['transition_digest'],
                    initial_snapshot_id=snapshot['snapshot_id'],valid=True,time_s=request['sample_times'],
                    T_world_object=poses,engine_domain='physics',engine='ManiSkill/PhysX CPU',
                    material_parameterization='shared_effective_object_support_tool_friction',
                    density_applied_at_collision_construction=True,
                    native_target_mass_kg=float(target_body.mass),
                    native_target_collision_shapes=len(target_body.collision_shapes),
                    native_tool_geometry_digest=None if native_program is None else native_program.geometry_digest,
                    native_tool_motion_digest=None if native_program is None else native_program.motion_digest,
                    native_tool_shape_count=0 if native_program is None else sum(map(len,native_program.links.values())),
                    measured_tool_feedback=dict(source="measured_feedback",time_s=feedback_times,
                        T_world_tcp=feedback_poses,stages=feedback_stages,
                        measurement_source="sapien_kinematic_actor_pose_after_step"),
                    safety_qualification=False,full_arm_simulated=False,
                    note='Measured tool replay for identification only; not execution path approval')
    finally:
        if env is not None:env.close()


def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--request',type=Path,required=True);p.add_argument('--result',type=Path,required=True)
    args=p.parse_args()
    if args.result.exists():raise FileExistsError('Do not overwrite a replay result')
    if args.request.stat().st_size>16000000:raise ValueError('Replay request too large')
    # Direct CLI use gets the same mandatory network boundary as pool workers.
    from tools.run_network_isolated import block_network
    block_network()
    output=physical_replay(json.loads(args.request.read_text()))
    args.result.write_text(json.dumps(output,allow_nan=False),encoding='utf-8')

if __name__=='__main__':main()
