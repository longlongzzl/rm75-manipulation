"""One push at a time, with post-motion observation and a finite stop policy."""
from __future__ import annotations
import time
from .model import Config, choose_push, error, reached


class PushTController:
    def __init__(self,observer,executor,config:Config,stop,events,*,real=False):
        self.observer=observer;self.executor=executor;self.config=config
        self.stop=stop;self.events=events;self.real=real

    def run(self,target):
        previous=None;after=0.;best=None;stagnant=0
        for step in range(self.config.max_steps+1):
            self.stop.check()
            obs=self.observer.observe(after=after)
            obs.validate(previous=previous,after=after,max_age_s=self.config.max_observation_age_s,real=self.real)
            previous=obs
            score=error(obs.pose,target,self.config)
            self.events.emit('observation',step=step,observation=obs.as_dict(),error=score)
            if reached(obs.pose,target,self.config):
                confirmed=True
                for _ in range(self.config.success_observations-1):
                    barrier=time.time()
                    self.stop.wait(self.config.success_dwell_s/(self.config.success_observations-1))
                    latest=self.observer.observe(after=barrier)
                    latest.validate(previous=obs,after=barrier,max_age_s=self.config.max_observation_age_s,real=self.real)
                    obs=latest;previous=latest
                    self.events.emit('goal_confirmation',observation=obs.as_dict())
                    if not reached(obs.pose,target,self.config):
                        confirmed=False;break
                if confirmed:
                    return {'command_success':True,'task_success':True,'verification':'live_pose' if self.real else 'surrogate_pose',
                            'steps':step,'final_observation':obs.as_dict(),'model_validated_on_robot':False,
                            'success_observations':self.config.success_observations}
                score=error(obs.pose,target,self.config)
            if step==self.config.max_steps:
                raise RuntimeError('maximum_push_count_reached')
            if best is None or score<best-self.config.minimum_improvement:
                best=score;stagnant=0
            else:
                stagnant+=1
                if stagnant>=self.config.stagnation_steps:
                    raise RuntimeError('stagnation_no_observed_progress')
            push,prediction=choose_push(obs.pose,target,self.config)
            self.events.emit('push_planned',step=step,push=push.as_dict(),prediction=prediction,
                             based_on={'session':obs.session_id,'sequence':obs.sequence})
            self.stop.check()
            obs.validate(max_age_s=self.config.max_observation_age_s,real=self.real)
            self.executor.execute_push(push,obs)
            after=time.time()  # Captured after all motion/settling callbacks return.
            self.events.emit('push_command_finished',step=step,finished_at=after)
