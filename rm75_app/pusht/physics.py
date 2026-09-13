"""Explicit physics observer/executor for the actual workcell worker.

Each push is newly planned from actual simulation state. Ground truth is labelled
simulation, not camera/live data; the hardware execute_push path is never called.
"""
from collections import deque
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import selectors
import subprocess
import tempfile
import time
from types import SimpleNamespace
import uuid
import numpy as np

from .observation import Observation
from .physics_replay import TcpFK,TimedProgram,TimedStageProgram
from .controller import PushTController
from .model import valid_pose
from rm75_app.workcell.io import atomic_json,dumps

ROOT=Path(__file__).resolve().parents[2]
# Bounded in-memory display cache; the full history stays on disk.
OBSERVATION_CACHE=4096
OBSERVATION_FLUSH_STEPS=30


def write_observation_array(path,stream_path):
    """Rebuild the original observations.json array from the streamed rows.

    Same bytes as one dumps(list) call, written row by row so a long session
    never has to be held in memory at once.
    """
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fd,temp=tempfile.mkstemp(prefix=f'.{path.name}.',dir=path.parent)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as out:
            with Path(stream_path).open('r',encoding='utf-8') as stream:
                out.write('[')
                first=True
                for line in stream:
                    line=line.strip()
                    if not line:continue
                    if not first:out.write(', ')
                    out.write(line);first=False
                out.write(']\n')
            out.flush();os.fsync(out.fileno())
        os.replace(temp,path)
    finally:
        if os.path.exists(temp):os.unlink(temp)


class PhysicsSession:
    def __init__(self,spec,section,config,stop,events):
        self.spec,self.section,self.config,self.stop,self.events=spec,section,config,stop,events
        self.kind=spec['parameters']['simulation_backend'];self.full_arm=self.kind=='full_arm_physics'
        self.directory=events.directory/'physics';self.directory.mkdir(exist_ok=False)
        self.process=None;self.env=None;self.video=None;self.stderr=None;self.frames=0;self.steps=0
        self.sequence=0;self.session=uuid.uuid4().hex;self.started=time.monotonic()
        self.feedback_action_id=None
        # Per-control-step ground truth streams to disk; only a bounded tail is
        # kept in memory, so run_until_goal sessions cannot grow an unbounded
        # array. close() rebuilds the original observations.json array.
        self.observations=deque(maxlen=OBSERVATION_CACHE)
        self.observations_recorded=0
        self.observation_log=(self.directory/'observations.jsonl').open('w',encoding='utf-8')
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
        self.planner_geometry=hello
        self.report['retreat_collision_checks']=hello['retreat_collision_checks']
        self.report['planned_gripper_joint_targets']=hello['gripper_locks']
        self.report['ignore_gripper_internal_self_collision']=hello.get('ignore_gripper_internal_self_collision',False)
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
        from rm75_app.swm.native_simulation_clock import NativeSimulationClock
        self.simulation_clock=NativeSimulationClock(self.base)
        import imageio.v2 as imageio
        self.video=imageio.get_writer(self.directory/'closed_loop.mp4',fps=10,codec='libx264',pixelformat='yuv420p')
        self.advance(2.)
        if self.full_arm:
            from .swm_capture import PushTSceneCapture
            self.swm_capture=PushTSceneCapture(self)
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

    def measured_tool_feedback(self):
        """Read native articulated feedback, including post-action observations.

        Raw evidence only: this is not an SWM receipt or fitted transition.
        The kinematic command surrogate is never labeled measured feedback.
        """
        if not self.full_arm:return None
        started=time.monotonic();clock=self.simulation_clock.read()
        def joints():
            return self.base.robot.get_qpos().detach().cpu().numpy().reshape(-1).copy()
        q=joints()
        tcp=self.base.tcp.pose.to_transformation_matrix().detach().cpu().numpy().reshape(4,4).copy()
        if not np.isfinite(q).all() or not np.isfinite(tcp).all():
            raise RuntimeError('Nonfinite articulated PushT feedback')
        if not np.array_equal(q,joints()) or self.simulation_clock.read()!=clock:
            raise RuntimeError('Articulated PushT state changed during feedback read')
        ended=time.monotonic()
        return dict(source='measured_feedback',domain='physics',
            actual_action_id=self.feedback_action_id,sensor_session=self.session,
            captured_at=started,query_finished_at=ended,query_elapsed_s=ended-started,
            simulation_clock=clock,frame_id='world',
            joint_names=[self.base.joint_names[i] for i in self.base.arm_indices],
            joint_positions=q[self.base.arm_indices].tolist(),
            gripper_joint_positions={n:float(q[i]) for n,i in self.base.gripper_indices.items()},
            T_world_tcp=tcp.tolist(),stage=self.base.stage,
            tcp_source='original_native_TCP_link_pose',hardware_connected=False)

    def advance(self,seconds):
        from PIL import Image,ImageDraw
        count=max(1,int(np.ceil(seconds*self.base.control_freq)))
        for _ in range(count):
            self.stop.check();self.env.step(None);self.steps+=1;self.report['physics_stepped']=True
            feedback=self.measured_tool_feedback()
            pose,state=self.physical_state()
            self.record_observation(dict(time_s=self.base.physics_time,stage=self.base.stage,
                pose=pose.tolist(),tool_feedback=feedback,**state))
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

    def record_observation(self,row):
        """Stream one control-step sample to disk and keep a bounded tail.

        The streamed rows are the durable evidence; the in-memory cache is only
        what a caller may inspect in process. Flushing is periodic because this
        runs once per control step of a long session.
        """
        self.observations.append(row);self.observations_recorded+=1
        self.observation_log.write(dumps(row)+'\n')
        if self.observations_recorded%OBSERVATION_FLUSH_STEPS==0:self.observation_log.flush()

    def observe(self,after=0.):
        # A new physics sample, not a republished pose with a newer wall timestamp.
        self.advance(1/self.base.control_freq)
        self.sequence+=1;pose,_=self.physical_state()
        return Observation(self.session,self.sequence,self.clock(),tuple(float(v) for v in pose),'simulation')

    def disturb(self,pose):
        """Explicit external-disturbance hook: overwrite the dynamic T pose.

        Only reachable from a disturbance schedule configured in the machine
        profile for dedicated recovery tests; the controller itself never
        teleports the target and the plan/execute path is untouched. The moved
        pose is re-observed as fresh physics ground truth before the next push.
        """
        import sapien
        before,_=self.physical_state()
        transform=self.base.target.pose.to_transformation_matrix().detach().cpu().numpy().reshape(4,4)
        moved=np.asarray([float(pose[0]),float(pose[1]),float(transform[2,3])],dtype=float)
        yaw=float(pose[2])
        self.base.target.set_pose(sapien.Pose(p=moved,q=[np.cos(yaw/2),0,0,np.sin(yaw/2)]))
        self.base.target.set_linear_velocity(np.zeros(3))
        self.base.target.set_angular_velocity(np.zeros(3))
        after,_=self.physical_state()
        if not valid_pose(after,self.config):
            raise ValueError('External disturbance pose leaves the T planar/support workspace')
        self.report.setdefault('disturbances',[]).append(dict(time_s=self.base.physics_time,
            before_pose=[float(v) for v in before],after_pose=[float(v) for v in after],
            kind='external_pose_overwrite_between_pushes'))
        self.events.emit('external_disturbance_applied',time_s=self.base.physics_time,
                         before_pose=before.tolist() if hasattr(before,'tolist') else list(before),
                         after_pose=after.tolist() if hasattr(after,'tolist') else list(after))
        return True

    def contact_direction_mask(self,obs,config):
        from .closed_gripper import ToolGeometry,contact_direction_mask
        from .motion import CuroboPushExecutor
        geometry=ToolGeometry(np.asarray(self.planner_geometry['spheres']),tuple(self.planner_geometry['links']))
        scene=CuroboPushExecutor(None,None,config,self.motion,self.stop,self.events,None)._scene(obs)
        return contact_direction_mask(obs,config,self.motion,geometry,scene)

    def prepare_push_candidates(self,proposals,obs,*,model=None):
        self.stop.check();obs.validate(now=self.clock(),max_age_s=self.config.max_observation_age_s)
        if obs.source!='simulation':raise ValueError('Physics executor requires simulation ground truth')
        q=self.base.read_q()
        feedback={}
        if self.full_arm:
            actual=self.base.robot.get_qpos().detach().cpu().numpy().reshape(-1)
            gap=max(abs(actual[i]-self.base.gripper_locks[n]) for n,i in self.base.gripper_indices.items())
            self.report['last_closed_gripper_joint_error_rad']=float(gap)
            if gap>.02:raise RuntimeError('Articulated gripper does not match the closed planning model')
            self.report['actual_gripper_joint_positions']={n:float(actual[i]) for n,i in self.base.gripper_indices.items()}
            if self.section['physics'].get('gripper_feedback_geometry') is True:
                feedback['simulated_gripper_joint_positions']=self.report['actual_gripper_joint_positions']
        self.process.stdin.write(json.dumps(dict(op='plan',q=q.tolist(),response_fits=list((model or self.config).response_fits),push_candidates=[p.as_dict() for p,_ in proposals],observation=obs.as_dict(),**feedback))+'\n')
        self.process.stdin.flush();reply=self.receive();path=Path(reply['result'])
        path.resolve().relative_to(self.directory.resolve());result=json.loads(path.read_text())
        self.report['plans'].append(dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            complete=result['complete_chain'],planning_wall_s=reply['elapsed_s'],observation_sequence=obs.sequence))
        self.events.emit('physics_replan',backend=self.kind,plan=self.report['plans'][-1],
            based_on_actual_q=True,based_on_actual_T_pose=True)
        if not result['complete_chain']:raise RuntimeError('Physics replanning rejected: '+result.get('error','unknown'))
        if result['source_observation']!=obs.as_dict():raise ValueError('Planner observation identity mismatch')
        selected=result['selected_candidate']
        if not 0<=selected<len(proposals) or result['selected_push']!=proposals[selected][0].as_dict():
            raise ValueError('Planner selected an unknown push candidate')
        program=TimedProgram(result,self.fk)
        # Measured evidence for the interactive path: an extra observation
        # advances the physics one control step, so the plan's start q and the
        # simulation q are compared and reported as numbers. The original 1e-5
        # gate below is unchanged: a real state change still fails the run.
        drift=float(np.max(abs(program.initial-self.base.read_q())))
        self.report['last_planning_start_q_drift_rad']=drift
        if drift>1e-5:raise ValueError('Simulated start changed during planning: '
                                       f'{drift:.3e} rad > 1e-5 rad')
        self._prepared_selection=(proposals[selected][0],obs.as_dict(),program)
        self._prepared_contact_binding=result.get('contact_binding')
        return selected

    def replan_measured_stage(self,stage_program,push,observation):
        q=self.base.read_q()
        payload=dict(op='stage',q=q.tolist(),stage=stage_program.hold_stage,
            goal_q=stage_program.positions[-1].tolist(),push=push.as_dict(),
            observation=observation.as_dict(),contact_binding=self._prepared_contact_binding)
        feedback=self.measured_tool_feedback()
        if feedback is not None:
            payload['simulated_gripper_joint_positions']=feedback['gripper_joint_positions']
        self.process.stdin.write(json.dumps(payload)+'\n');self.process.stdin.flush()
        reply=self.receive();path=Path(reply['result'])
        path.resolve().relative_to(self.directory.resolve());result=json.loads(path.read_text())
        if (result.get('stage_validated') is not True or result.get('stage')!=stage_program.hold_stage
                or result.get('source_observation')!=observation.as_dict()
                or result.get('measured_start_q')!=q.tolist()):
            raise RuntimeError('Measured stage planning rejected: '+str(result.get('error','identity mismatch')))
        self.events.emit('physics_measured_stage_plan',stage=stage_program.hold_stage,
            actual_action_id=self.feedback_action_id,observation_sequence=observation.sequence,
            file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            swm_scene_audit_verified=False)
        return TimedStageProgram(stage_program.hold_stage,np.asarray(result['positions']),
                                 np.asarray(result['times']),self.fk)

    def check_stage_start(self,stage_program):
        """Fresh read after boundary capture, before installing any new command."""
        self.stop.check()
        actual=np.asarray(self.base.read_q(),dtype=float)
        expected=np.asarray(stage_program.initial,dtype=float)
        if (actual.shape!=(7,) or expected.shape!=(7,) or
                not np.isfinite(actual).all() or not np.isfinite(expected).all()):
            raise ValueError('Invalid stage-start joint feedback')
        drift=float(np.max(abs(actual-expected)))
        self.report['last_stage_start_check']=dict(stage=stage_program.hold_stage,
            drift_rad=drift,limit_rad=1e-5,after_boundary_capture=True)
        if drift>1e-5:
            raise ValueError('Simulated stage start changed after boundary capture: '
                             f'{drift:.3e} rad > 1e-5 rad')

    def execute_push(self,push,obs):
        self.stop.check();obs.validate(now=self.clock(),max_age_s=self.config.max_observation_age_s)
        cached=getattr(self,'_prepared_selection',None)
        if cached is None or cached[0]!=push or cached[1]!=obs.as_dict():
            self.prepare_push_candidates([(push,{})],obs)
            cached=self._prepared_selection
        self._prepared_selection=None;program=cached[2]
        # Same measured drift, now with the interactive pre-execution observation
        # already applied; the gate is the original 1e-5 radians, unchanged.
        drift=float(np.max(abs(program.initial-self.base.read_q())))
        self.report['last_pre_execution_start_q_drift_rad']=drift
        if drift>1e-5:raise ValueError('Simulated start changed after planning: '
                                       f'{drift:.3e} rad > 1e-5 rad')
        self.feedback_action_id=uuid.uuid4().hex
        if self.full_arm:self.last_swm_before=self.swm_capture.capture('before_push')
        self.events.emit('physics_action_started',actual_action_id=self.feedback_action_id,
            observation_sequence=obs.sequence,measured_tool_feedback=self.measured_tool_feedback())
        for stage_program in program.stage_programs():
            # A clamped stage cannot advance into retreat while a boundary
            # observation is being collected. Keep original stage contact rules
            # during holds, rather than relabeling every hold as post_settle.
            before=self.observe()
            self.events.emit('physics_stage_boundary',boundary='before',
                stage=stage_program.hold_stage,actual_action_id=self.feedback_action_id,
                observation=before.as_dict(),tool_feedback=self.measured_tool_feedback(),
                swm_scene_audit_verified=False)
            stage_program=self.replan_measured_stage(stage_program,push,before)
            self.check_stage_start(stage_program)
            self.base.active_program=stage_program;self.base.program_started=self.base.physics_time
            self.advance(stage_program.duration)
            after=self.observe(after=before.captured_at)
            self.events.emit('physics_stage_boundary',boundary='after',
                stage=stage_program.hold_stage,actual_action_id=self.feedback_action_id,
                observation=after.as_dict(),tool_feedback=self.measured_tool_feedback(),
                swm_scene_audit_verified=False)
        self.advance(1.)
        if self.full_arm:
            gap=float(np.max(abs(self.base.read_q()-program.rows[-1][2][-1])))
            self.report['last_final_joint_tracking_error_rad']=gap
            if gap>.02:raise RuntimeError('Articulated final joint tracking error')
            self.last_swm_after=self.swm_capture.capture('after_push')

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
        self.observation_log.flush();self.observation_log.close()
        self.report['observation_samples']=self.observations_recorded
        self.report['observation_display_cache_rows']=len(self.observations)
        write_observation_array(self.directory/'observations.json',self.directory/'observations.jsonl')
        atomic_json(self.directory/'summary.json',self.report)


def run(spec,profile,config,stop,events):
    if spec['mode']!='sim':raise PermissionError('Physics cannot execute in real mode')
    session=PhysicsSession(spec,profile['pusht'],config,stop,events)
    try:
        session.open()
        schedule=list(profile['pusht'].get('physics',{}).get('disturbances') or [])
        session.report['disturbance_schedule']=schedule
        def disturb(step):
            for entry in schedule:
                if int(entry.get('after_pushes',-1))==step:
                    session.disturb(tuple(entry['pose']))
                    return True
            return False
        result=PushTController(session,session,config,stop,events,clock=session.clock,
            wait=session.advance,verification='physics_pose',
            disturb=disturb if schedule else None).run(spec['parameters']['goal_pose'])
        result.update(simulation_backend=session.kind,hardware_connected=False,hardware_profile_qualified=False)
        session.report['task_success']=result['task_success'];return result
    except BaseException as exc:
        session.report.update(task_success=False,error=f'{type(exc).__name__}: {exc}')
        raise
    finally:session.close()
