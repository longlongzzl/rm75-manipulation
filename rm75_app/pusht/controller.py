"""One push at a time, with post-motion observation and a finite stop policy."""
from __future__ import annotations
import time
from .model import Config, error, reached
from .batch_search import rank_pushes


class PushTController:
    def __init__(self,observer,executor,config:Config,stop,events,*,real=False,
                 clock=None,wait=None,verification=None):
        self.observer=observer;self.executor=executor;self.config=config
        self.stop=stop;self.events=events;self.real=real
        self.clock=clock or time.time;self.wait=wait or stop.wait
        self.verification=verification or ('live_pose' if real else 'surrogate_pose')
        if self.verification not in ('live_pose','surrogate_pose','physics_pose'):
            raise ValueError('Unknown observation verification domain')
        if real != (self.verification=='live_pose'):
            raise ValueError('Physics/surrogate verification cannot authorize real execution')

    def run(self,target):
        from .response import ResponseEstimator
        response=ResponseEstimator(self.config);pending_response=None
        previous=None;after=0.;best=None;stagnant=0
        for step in range(self.config.max_steps+1):
            self.stop.check()
            obs=self.observer.observe(after=after)
            obs.validate(now=self.clock(),previous=previous,after=after,max_age_s=self.config.max_observation_age_s,real=self.real)
            previous=obs
            if pending_response is not None:
                before,executed=pending_response
                fit=response.update(before,executed,obs.pose);pending_response=None
                self.events.emit('push_response_fitted',fit=fit,source=self.verification,
                                 observation_sequence=obs.sequence)
            score=error(obs.pose,target,self.config)
            self.events.emit('observation',step=step,observation=obs.as_dict(),error=score)
            if reached(obs.pose,target,self.config) and getattr(self.observer,'stable',lambda:True)():
                confirmed=True
                for _ in range(self.config.success_observations-1):
                    barrier=self.clock()
                    self.wait(self.config.success_dwell_s/(self.config.success_observations-1))
                    latest=self.observer.observe(after=barrier)
                    latest.validate(now=self.clock(),previous=obs,after=barrier,max_age_s=self.config.max_observation_age_s,real=self.real)
                    obs=latest;previous=latest
                    self.events.emit('goal_confirmation',observation=obs.as_dict())
                    if not reached(obs.pose,target,self.config) or not getattr(self.observer,'stable',lambda:True)():
                        confirmed=False;break
                if confirmed:
                    return {'command_success':True,'task_success':True,'verification':self.verification,
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
            search_config=response.config()
            geometry_filter=getattr(self.executor,'contact_direction_mask',None)
            mask=geometry_filter(obs,search_config) if geometry_filter is not None else None
            proposals=rank_pushes(obs.pose,target,search_config,limit=128,contact_mask=mask)
            prepare=getattr(self.executor,'prepare_push_candidates',None)
            self.stop.check()
            # Prepared execution checks a new live pose after planning, before motion.
            if prepare is None:
                obs.validate(now=self.clock(),max_age_s=self.config.max_observation_age_s,real=self.real)
            selected=prepare(proposals,obs,model=search_config) if prepare is not None else 0
            push,prediction=proposals[selected]
            self.events.emit('push_candidates_ranked',step=step,selected_rank=selected,
                candidates=[dict(push=p.as_dict(),predicted_cost=r['predicted_cost']) for p,r in proposals],
                evaluated_friction_rollouts=prediction['evaluated_friction_rollouts'])
            self.events.emit('push_planned',step=step,push=push.as_dict(),prediction=prediction,
                             based_on={'session':obs.session_id,'sequence':obs.sequence})
            self.stop.check()
            if prepare is None:
                obs.validate(now=self.clock(),max_age_s=self.config.max_observation_age_s,real=self.real)
            self.executor.execute_push(push,obs)
            pending_response=(obs.pose,push)
            after=self.clock()  # Captured after all motion/settling callbacks return.
            self.events.emit('push_command_finished',step=step,finished_at=after)
