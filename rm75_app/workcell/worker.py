"""Isolated task worker. Runs original engines or the new PushT loop."""
from __future__ import annotations
import argparse
import traceback
import time
import uuid
from dataclasses import asdict
from pathlib import Path
from .io import read_json,atomic_json,digest
from .events import EventLog,StopToken,Cancelled
from .locking import ResourceLease
from .spec import validate_spec


def run_pusht(spec,profile,stop,events):
    from rm75_app.pusht.model import Config,predict,choose_push
    from rm75_app.pusht.observation import Observation,JsonObserver,AprilTagObserver
    from rm75_app.pusht.controller import PushTController
    params=spec['parameters'];cfg=dict(profile.get('pusht',{}).get('model',{}))
    cfg.update({key:params[key] for key in ('speed_mps','max_steps')})
    config=Config.from_dict(cfg)
    real=spec['mode']=='real'
    arm=None;backend=None;observer=None
    try:
        if not real:
            class Surrogate:
                def __init__(self):
                    self.pose=params['initial_pose'];self.seq=0;self.session=uuid.uuid4().hex
                def observe(self,after=0.):
                    self.seq+=1
                    return Observation(self.session,self.seq,time.time(),tuple(self.pose),'surrogate')
                def execute_push(self,push,obs):
                    stop.check();self.pose=predict(self.pose,push,config)
                    stop.wait(.03)
                def close(self):
                    pass
            observer=Surrogate();executor=observer
            events.emit('simulation_warning',message='CPU surrogate regression only; NOT ManiSkill or physical fidelity validation')
        else:
            from rm75_app.workcell.realman import RealManArm
            from rm75_app.planning.backends.curobo2 import Curobo2Backend,Curobo2BackendConfig
            from rm75_app.pusht.motion import CuroboPushExecutor
            section=profile['pusht'];camera=section['observer']
            observer=(AprilTagObserver(camera,stop) if camera['kind']=='realsense_apriltag'
                      else JsonObserver(camera['observation_file'],stop) if camera['kind']=='json_live' else None)
            if observer is None:
                raise ValueError('Unknown live PushT observer')
            # Prove valid live perception before opening the arm control connection.
            observer.observe().validate(max_age_s=config.max_observation_age_s,real=True)
            options=dict(section.get('planner',{}))
            for key in ('robot_config','curobo_root'):
                if key in options:
                    options[key]=Path(options[key]).expanduser().resolve()
            backend=Curobo2Backend(Curobo2BackendConfig(**options))
            arm=RealManArm(profile['hardware'],stop,events)
            executor=CuroboPushExecutor(backend,arm,config,section['motion'],stop,events,observer)
        return PushTController(observer,executor,config,stop,events,real=real).run(params['goal_pose'])
    except BaseException:
        if arm is not None:
            try:
                arm.controlled_stop()
            except Exception as exc:
                events.emit('stop_failed',error=str(exc),requires_physical_estop=True)
        raise
    finally:
        if arm is not None:
            arm.close()
        if backend is not None:
            backend.__exit__(None,None,None)
        if observer is not None:
            observer.close()


def preview(spec,profile):
    if spec['task']=='magnetic':
        from rm75_app.magnetic.design import validate_design
        return {'command_success':True,'task_success':None,'verification':'preview_only',
                'design':validate_design(spec['parameters']['design']).report()}
    if spec['task']=='pusht':
        from rm75_app.pusht.model import Config,choose_push,reached
        cfg=dict(profile.get('pusht',{}).get('model',{}));cfg['speed_mps']=spec['parameters']['speed_mps']
        model=Config.from_dict(cfg);p=spec['parameters']
        if reached(p['initial_pose'],p['goal_pose'],model):
            return {'command_success':True,'task_success':None,'verification':'preview_only','already_at_goal':True}
        push,prediction=choose_push(p['initial_pose'],p['goal_pose'],model)
        return {'command_success':True,'task_success':None,'verification':'preview_only',
                'push':push.as_dict(),'prediction':prediction,'geometry':asdict(model)}
    return {'command_success':True,'task_success':None,'verification':'preview_only',
            'object_name':spec['parameters']['object_name'],'planner':'preserved_working_pickplace',
            'placement':'original_object_specific_place_rules','hardware_connected':False}


def main(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--run-dir',required=True,type=Path)
    parser.add_argument('--profile',required=True,type=Path)
    parser.add_argument('--app-root',required=True,type=Path)
    parser.add_argument('--real-authorized',action='store_true')
    args=parser.parse_args(argv);run_dir=args.run_dir.resolve()
    events=EventLog(run_dir);stop=StopToken(run_dir/'STOP')
    profile=read_json(args.profile);spec=validate_spec(read_json(run_dir/'request.json'),profile)
    result=None
    try:
        if spec['mode']=='real':
            if not args.real_authorized or profile.get('hardware',{}).get('hardware