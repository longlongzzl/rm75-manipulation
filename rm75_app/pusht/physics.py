"""Explicit physics observer/executor for the actual workcell worker.

Each push is newly planned from actual simulation state. Ground truth is labelled
simulation, not camera/live data; the hardware execute_push path is never called.
"""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import selectors
import subprocess
import time
from types import SimpleNamespace
import uuid
import numpy as np

from .observation import Observation
from .physics_replay import TcpFK,TimedProgram
from .controller import PushTController
from .model import valid_pose
from rm75_app.workcell.io import atomic_json

ROOT=Path(__file__).resolve().parents[2]


class PhysicsSession:
    def __init__(self,spec,section,config,stop,events):
        self.spec,self.section,self.config,self.stop,self.events=spec,section,config,stop,events
        self.kind=spec['parameters']['simulation_backend'];self.full_arm=self.kind=='full_arm_physics'
        self.directory=events.directory/'physics';self.directory.mkdir(exist_ok=False)
        self.process=None;self.env=None;self.video=None;self.stderr=None;self.frames=0;self.steps=0
        self.sequence=0;self.session=uuid.uuid4().hex;self.started=time.monotonic();self.observations=[]
        self.report=dict(backend=self.kind,execute_real=False,hardware_connected=False,
            hardware_profile_qualified=False,model_validated_on_robot=False,arm_servo_simulated=self.full_arm,
            target_driven_by_physics_only=True,physics_stepped=False,plans=[],
            materials_calibrated=False,physics_clock_contract='1 + stepped simulation seconds; planning wall time recorded separately',
            full_arm_collision_geometry='unchanged original URDF meshes; no duplicate kinematic tool' if self.full_arm else 'original 38 closed-tool spheres')

    def receive(self,timeout=180):
        selector=selectors.DefaultSelector();selector.register(self.process.stdout,selectors.EVENT_READ)
        deadline=time.monotonic()+timeout
        try:
            while time.monotonic()<deadline:
                self.stop.check()
                if selector.select(.05):
                    line=self.process.stdout.readline()
                    if not line:raise RuntimeError('Physics planner exited: '+str(self.process.poll()))
                    value=json.loads(line)
                    if value.get('error') and value.get('ready') is False:raise RuntimeError(value['error'])
                    return value
            raise TimeoutError('Physics planner response timeout')
        finally:selector.close()

    def open(self):
        physics=self.section['physics'];motion=physics['motion'];self.motion=motion
        self.report['arm_gravity_compensation']=physics.get('gravity_compensation','none')
        self.report['target_gravity_enabled']=True
        self.report['drive_source']='Beta_demo-codex-v0.9/jimu_portable_repro/maniskill_env/mani_skill/agents/robots/realman/realman_with_gripper.py'
        drive_source=ROOT/'rm75_app/_vendor/working_snapshot'/self.report['drive_source']
        self.report['drive_source_sha256']=hashlib.sha256(drive_source.read_bytes()).hexdigest()
        self.report['drive_constants']=dict(arm_stiffness=1000.,arm_damping=100.,arm_force_limit=20.,
            arm_friction=.1,gripper_stiffness=1000.,gripper_damping=100.,gripper_force_limit=5.,gripper_friction=1.)
        data=dict(model=asdict(self.config),motion=motion,planner=physics.get('planner',{}))
        atomic_json(self.directory/'planner_profile.json',data)
        self.stderr=(self.directory/'planner.stderr.log').open('w')
        self.process=subprocess.Popen([physics['planner_python'],'-u',str(ROOT/'tools/pusht_physics_planner.py'),
            '--profile',str(self.directory/'planner_profile.json'),'--directory',str(self.directory)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.stderr,text=True,bufsize=1)
        hello=self.receive();atomic_json(self.directory/'planner_geometry.json',hello)
        self.report['planned_gripper_joint_targets']=hello['gripper_locks']
        from rm75_app.planning.gripper_collision import gripper_link_transforms
        jaw_values=set(hello['gripper_locks'].values())
        if len(jaw_values)!=1:raise ValueError('Coupled jaw model has inconsistent joint targets')
        urdf=ROOT/'assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf'
        if urdf.read_bytes()!=Path(hello['urdf']).read_bytes():raise ValueError('Physics and GPU URDF differ')
        self.fk=TcpFK(urdf);self.report['urdf_sha256']=hashlib.sha256(urdf.read_bytes()).hexdigest()
        transforms=gripper_link_transforms(urdf,next(iter(jaw_values)))
        self.report['nominal_pad_reference_separation_m']=float(np.linalg.norm(
            transforms['left_pad'][:3,3]-transforms['right_pad'][:3,3]))
        import gymnasium as gym
        import rm75_app.simulation.pusht_loop_env
        from rm75_app.planning.contracts import CollisionObject,Pose
        initial=self.spec['parameters']['initial_pose']
        program=SimpleNamespace(initial=np.asarray(hello['initial_q']),fk=self.fk,duration=0.)
        self.env=gym.make('RM75-PushT-Loop-v1',program=program,spheres=hello['spheres'],config=self.config,
            observation=Observation(self.session,0,1.,tuple(initial),'simulation'),motion=motion,
            static_objects=[CollisionObject(o['name'],o['kind'],Pose(o['position'],o['quaternion_wxyz']),
                dimensions=o['dimensions']) for o in motion['static_collision_objects']],
            full_arm=self.full_arm,urdf=str(urdf),gripper_locks=hello['gripper_locks'],
            gravity_compensation=physics.get('gravity_compensation','none'),
            obs_mode='none',reward_mode='none',render_mode='rgb_array',sim_backend='physx_cpu',num_envs=1)
        self.base=self.env.unwrapped;self.env.reset(seed=0)
        import imageio.v2 as imageio
        self.video=imageio.get_writer(self.directory/'closed_loop.mp4',fps=10,codec='libx264',pixelformat='yuv420p')
        self.advance(2.)
        self.events.emit('physics_backend_ready',backend=self.kind,hardware_connected=False,
            observer='simulator_ground_truth',simulation_time_s=self.base.physics_time)

    def clock(self):return 1.+self.base.physics_time

    def physical_state(self):
        transform=self.base.target.pose.to_transformation_matrix().detach().cpu().numpy().reshape(4,4)
        pose=np.array([*transform[:2,3],np.arctan2(transform[1,0],transform[0,0])],dtype=float)
        linear=self.base.target.linear_velocity.detach().cpu().numpy().reshape(-1)
        angular=self.base.target.angular_velocity.detach().cpu().numpy().reshape(-1)
        return pose,dict(z_m=float(transform[2,3]),tilt_rad=float(np.arccos(np.clip(transform[2,2],-1,1))),
                         linear_speed_mps=float(np.linalg.norm(linear)),angular_speed_rad_s=float(np.linalg.norm(angular)))

    def stable(self):
        _,state=self.physical_state()
        return state['linear_speed_mps']<=.002 and state['angular_speed_rad_s']<=.02

    def advance(self,seconds):
        from PIL import Image,ImageDraw
        count=max(1,int(np.ceil(seconds*self.base.control_freq)))
        for _ in range(count):
            self.stop.check();self.env.step(None);self.steps+=1;self.report['physics_stepped']=True
            pose,state=self.physical_state()
            self.observations.append(dict(time_s=self.base.physics_time,stage=self.base.stage,
                pose=pose.tolist(),**state))
            if not np.isfinite(pose).all() or not all(np.isfinite(x) for x in state.values()):
                raise RuntimeError('Nonfinite physics feedback')
            if not valid_pose(pose,self.config) or abs(state['z_m']-self.motion['object_centroid_z_m'])>.01 or state['tilt_rad']>.10:
                raise RuntimeError('Physical T left planar/support workspace')
            if self.video is not None and self.steps%3==0:
                pixels=self.env.render()
                if hasattr(pixels,'detach'):pixels=pixels.detach().cpu().numpy()
                pixels=np.asarray(pixels)
                while pixels.ndim>3:pixels=pixels[0]
                canvas=Image.new('RGB',(pixels.shape[1],pixels.shape[0]+48),(17,22,30))
                canvas.paste(Image.fromarray(pixels[...,:3].astype(np.uint8)),(0,48))
                draw=ImageDraw.Draw(canvas)
                draw.text((6,5),f'{self.kind} | {self.base.stage} | physics t={self.base.physics_time:.2f}s | plans={len(self.report["plans"])}',fill='white')
                draw.text((6,24),'Dynamic T ground truth; uncalibrated material; NO hardware qualification',fill='white')
                self.video.append_data(np.asarray(canvas));self.frames+=1
            if self.base.early_contact_steps or self.base.obstacle_contact_steps:
                raise RuntimeError('Physics forbidden early-target or static-obstacle contact')
            if self.full_arm and self.base.forbidden_target_contacts:
                raise RuntimeError('Physics target contact by unapproved robot link/phase')

    def observe(self,after=0.):
        # A new physics sample, not a republished pose with a newer wall timestamp.
        self.advance(1/self.base.control_freq)
        self.sequence+=1;pose,_=self.physical_state()
        return Observation(self.session,self.sequence,self.clock(),tuple(float(v) for v in pose),'simulation')

    def execute_push(self,push,obs):
        self.stop.check();obs.validate(now=self.clock(),max_age_s=self.config.max_observation_age_s)
        if obs.source!='simulation':raise ValueError('Physics executor requires simulation ground truth')
        q=self.base.read_q()
        if self.full_arm:
            actual=self.base.robot.get_qpos().detach().cpu().numpy().reshape(-1)
            gap=max(abs(actual[i]-self.base.gripper_locks[n]) for n,i in self.base.gripper_indices.items())
            self.report['last_closed_gripper_joint_error_rad']=float(gap)
            if gap>.02:raise RuntimeError('Articulated gripper does not match the closed planning model')
        self.process.stdin.write(json.dumps(dict(op='plan',q=q.tolist(),push=asdict(push),observation=obs.as_dict()))+'\n')
        self.process.stdin.flush();reply=self.receive();path=Path(reply['result'])
        path.resolve().relative_to(self.directory.resolve());result=json.loads(path.read_text())
        self.report['plans'].append(dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            complete=result['complete_chain'],planning_wall_s=reply['elapsed_s'],observation_sequence=obs.sequence))
        self.events.emit('physics_replan',backend=self.kind,plan=self.report['plans'][-1],
            based_on_actual_q=True,based_on_actual_T_pose=True)
        if not result['complete_chain']:raise RuntimeError('Physics replanning rejected: '+result.get('error','unknown'))
        if result['source_observation']!=obs.as_dict():raise ValueError('Planner observation identity mismatch')
        program=TimedProgram(result,self.fk)
        if np.max(abs(program.initial-self.base.read_q()))>1e-5:raise ValueError('Simulated start changed during planning')
        self.base.active_program=program;self.base.program_started=self.base.physics_time
        self.advance(program.duration);self.advance(1.)
        if self.full_arm:
            gap=float(np.max(abs(self.base.read_q()-program.rows[-1][2][-1])))
            self.report['last_final_joint_tracking_error_rad']=gap
            if gap>.02:raise RuntimeError('Articulated final joint tracking error')

    def close(self):
        if self.video is not None:self.video.close()
        if self.env is not None:
            self.report.update(simulated_time_s=self.base.physics_time,physics_control_steps=self.steps,
                target_contact_substeps=len(self.base.contacts),early_target_contact_substeps=self.base.early_contact_steps,
                static_obstacle_contact_substeps=self.base.obstacle_contact_steps,
                max_joint_tracking_error_rad=max(self.base.tracking,default=0.) if self.full_arm else None)
            if self.full_arm:
                self.report.update(self_contact_pairs=self.base.self_contact_pairs,
                    static_contact_pairs=self.base.static_contact_pairs,
                    forbidden_target_contacts=self.base.forbidden_target_contacts,
                    target_contact_audit_scope='all articulated links against configured pusher_contact_links',
                    max_tcp_tracking_position_m=max((r[0] for r in self.base.tcp_tracking),default=0.),
                    max_tcp_tracking_angle_rad=max((r[1] for r in self.base.tcp_tracking),default=0.),
                    max_joint_velocity_rad_s=self.base.qvel_peak)
            atomic_json(self.directory/'contacts.json',self.base.contacts)
            self.env.close()
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=5)
            self.process.stdout.close();self.process.stdin.close()
        if self.stderr is not None:self.stderr.close()
        self.report.update(elapsed_wall_s=time.monotonic()-self.started,video_frames=self.frames)
        atomic_json(self.directory/'observations.json',self.observations)
        atomic_json(self.directory/'summary.json',self.report)


def run(spec,profile,config,stop,events):
    if spec['mode']!='sim':raise PermissionError('Physics cannot execute in real mode')
    session=PhysicsSession(spec,profile['pusht'],config,stop,events)
    try:
        session.open()
        result=PushTController(session,session,config,stop,events,clock=session.clock,
            wait=session.advance,verification='physics_pose').run(spec['parameters']['goal_pose'])
        result.update(simulation_backend=session.kind,hardware_connected=False,hardware_profile_qualified=False)
        session.report['task_success']=result['task_success'];return result
    except BaseException as exc:
        session.report.update(task_success=False,error=f'{type(exc).__name__}: {exc}')
        raise
    finally:session.close()
