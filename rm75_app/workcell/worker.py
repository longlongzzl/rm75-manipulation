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
            if not args.real_authorized or profile.get('hardware',{}).get('hardware_reviewed') is not True:
                raise PermissionError('Real execution is not authorized')
            if profile.get(spec['task'],{}).get('integration_qualified') is not True:
                raise PermissionError('Task integration has not passed local hardware qualification')
        with ResourceLease(args.app_root/'runtime_data'/'workcell'/'robot.lock'):
            stop.check();events.emit('task_started',task=spec['task'],mode=spec['mode'])
            if spec['mode']=='preview':
                result=preview(spec,profile)
            elif spec['task']=='pusht':
                result=run_pusht(spec,profile,stop,events)
            else:
                from .legacy import run_working
                result=run_working(spec,profile,args.app_root,run_dir,stop,events)
                verify=profile.get(spec['task'],{}).get('post_verification')
                if spec['mode']=='real' and verify:
                    from .verification import wait_for_poses
                    if verify.get('task_request_digest')!=digest(spec):
                        raise ValueError('Verification targets are not bound to this exact task request')
                    result.update(wait_for_poses(verify,after=time.time(),stop=stop))
            result['status']=('succeeded' if result.get('task_success') is True
                              else 'verification_failed' if result.get('task_success') is False
                              else 'command_completed_unverified')
    except Cancelled as exc:
        result={'status':'cancelled','command_success':False,'task_success':None,'error':str(exc)}
    except BaseException as exc:
        result={'status':'failed','command_success':False,'task_success':None,
                'error':f'{type(exc).__name__}: {exc}'}
        (run_dir/'traceback.txt').write_text(traceback.format_exc(),encoding='utf-8')
    finally:
        result.update({'finished_at':time.time(),'task':spec['task'],'mode':spec['mode']})
        atomic_json(run_dir/'result.json',result);events.emit('task_finished',result=result)
    return 1 if result['status'] in ('failed','verification_failed') else 0

if __name__=='__main__':
    raise SystemExit(main())
