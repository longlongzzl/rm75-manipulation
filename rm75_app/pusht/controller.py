"""Rank long/short pushes, execute one complete chain, then use a NEW observation.

Interactive sessions survive external scene changes and wait on stagnation,
not on unsafe motion. Solver/controller/collision faults are never swallowed.
"""
from __future__ import annotations
import time
from .model import Config, error, reached
from .session_control import SessionControl, moved, response_is_plausible, invalidate_prepared


def rank_pushes(*args, **kwargs):
    from .batch_search import rank_pushes as implementation
    return implementation(*args, **kwargs)


class PushTController:
    def __init__(self, observer, executor, config: Config, stop, events, *, real=False,
                 clock=None, wait=None, verification=None, control_policy=None):
        self.observer=observer; self.executor=executor; self.config=config
        self.stop=stop; self.events=events; self.real=real
        self.clock=clock or time.time; self.wait=wait or stop.wait
        self.verification=verification or ('live_pose' if real else 'surrogate_pose')
        if self.verification not in ('live_pose', 'surrogate_pose', 'physics_pose'):
            raise ValueError('Unknown observation verification domain')
        if real != (self.verification == 'live_pose'):
            raise ValueError('Physics/surrogate verification cannot authorize real execution')
        self.control=SessionControl(events, control_policy)
        self.interactive=(control_policy is not None or self.control.root is not None and
                          (self.control.root/'session_policy.json').is_file())

    def _observe(self, after, previous):
        self.stop.check(); self.control.check_budget()
        obs=self.observer.observe(after=after)
        obs.validate(now=self.clock(), previous=previous, after=after,
                     max_age_s=self.config.max_observation_age_s, real=self.real)
        return obs

    def _pause_boundary(self):
        changed=self.control.poll(self.executor, verification=self.verification) if self.interactive else False
        while self.control.paused:
            self.stop.check(); self.control.check_budget()
            self.control.state('paused')
            self.wait(self.control.policy.poll_s)
            if self.verification == 'physics_pose': self.stop.wait(.05)
            changed=self.control.poll(self.executor, verification=self.verification) or changed
        return changed

    def _stable_observation(self, obs):
        """After an intervention, do not repeatedly plan while the T is moving."""
        count=0
        self.control.state('waiting_for_stability', pose=obs.pose)
        while count < self.config.success_observations:
            self._pause_boundary()
            self.wait(self.control.policy.poll_s)
            if self.verification == 'physics_pose': self.stop.wait(.02)
            latest=self._observe(obs.captured_at, obs)
            stable=getattr(self.observer, 'stable', lambda: True)()
            count=count+1 if stable and not moved(obs.pose, latest.pose, self.control.policy) else 0
            obs=latest
        return obs

    def _wait_for_change(self, obs):
        """No new GPU plans while an unchanged scene has no observed progress."""
        self.control.state('waiting_for_scene_change', pose=obs.pose,
                           reason='stagnation; move T after pausing, or explicitly resume')
        initial=obs.pose; epoch=self.control.epoch
        while True:
            self.stop.check(); self.control.check_budget()
            self._pause_boundary()
            self.wait(self.control.policy.poll_s)
            if self.verification == 'physics_pose': self.stop.wait(.05)
            latest=self._observe(obs.captured_at, obs)
            obs=latest
            if self.control.epoch != epoch or moved(initial, obs.pose, self.control.policy):
                self.control.epoch+=1
                return self._stable_observation(obs)

    def run(self, target):
        from .response import ResponseEstimator
        response=ResponseEstimator(self.config); pending_response=None
        previous=None; after=0.; best=None; stagnant=0; step=0; need_stability=False
        try:
            while True:
                self.stop.check(); self.control.check_budget()
                if self._pause_boundary():
                    pending_response=None; best=None; stagnant=0; need_stability=True
                    self.events.emit('push_response_discarded', reason='explicit_operator_intervention')
                obs=self._observe(after, previous)
                if need_stability:
                    obs=self._stable_observation(obs); need_stability=False
                previous=obs
                if pending_response is not None:
                    before, executed, epoch=pending_response; pending_response=None
                    if response_is_plausible(before, executed, obs.pose, intervention=epoch != self.control.epoch):
                        fit=response.update(before, executed, obs.pose)
                        self.events.emit('push_response_fitted', fit=fit, source=self.verification,
                                         observation_sequence=obs.sequence)
                    else:
                        best=None; stagnant=0; self.control.epoch+=1
                        self.events.emit('push_response_discarded', reason='external_or_unexplained_motion',
                                         detector_guarantees_all_interventions=False)
                score=error(obs.pose, target, self.config)
                self.events.emit('observation', step=step, observation=obs.as_dict(), error=score)
                self.control.state('observing', step=step, pose=obs.pose)
                if reached(obs.pose, target, self.config) and getattr(self.observer, 'stable', lambda: True)():
                    confirmed=True; epoch=self.control.epoch
                    for _ in range(self.config.success_observations-1):
                        barrier=self.clock()
                        self.wait(self.config.success_dwell_s/(self.config.success_observations-1))
                        if self._pause_boundary() or epoch != self.control.epoch:
                            confirmed=False; need_stability=True; break
                        latest=self._observe(barrier, obs)
                        obs=latest; previous=latest
                        self.events.emit('goal_confirmation', observation=obs.as_dict())
                        if not reached(obs.pose, target, self.config) or not getattr(self.observer, 'stable', lambda: True)():
                            confirmed=False; break
                    if confirmed:
                        self.control.state('succeeded', pose=obs.pose)
                        return dict(command_success=True, task_success=True, verification=self.verification,
                            steps=step, final_observation=obs.as_dict(), model_validated_on_robot=False,
                            success_observations=self.config.success_observations,
                            external_scene_epochs=self.control.epoch)
                    score=error(obs.pose, target, self.config)
                if step >= self.config.max_steps and not self.control.policy.run_until_goal:
                    raise RuntimeError('maximum_push_count_reached')
                if best is None or score < best-self.config.minimum_improvement:
                    best=score; stagnant=0
                else:
                    stagnant+=1
                    if stagnant >= self.config.stagnation_steps:
                        if not self.control.policy.run_until_goal:
                            raise RuntimeError('stagnation_no_observed_progress')
                        obs=self._wait_for_change(obs); previous=obs; after=obs.captured_at
                        best=None; stagnant=0; pending_response=None
                        invalidate_prepared(self.executor)
                        continue
                search_config=response.config()
                geometry_filter=getattr(self.executor, 'contact_direction_mask', None)
                mask=geometry_filter(obs, search_config) if geometry_filter is not None else None
                try:
                    proposals=rank_pushes(obs.pose, target, search_config, limit=128, contact_mask=mask)
                except RuntimeError as exc:
                    if not self.control.policy.run_until_goal or str(exc) != 'no_valid_future':
                        raise
                    obs=self._wait_for_change(obs); previous=obs; after=obs.captured_at
                    best=None; stagnant=0; pending_response=None
                    invalidate_prepared(self.executor)
                    continue
                if not proposals: raise RuntimeError('no_valid_future')
                prepare=getattr(self.executor, 'prepare_push_candidates', None)
                self.stop.check(); self.control.state('planning', pose=obs.pose)
                if prepare is None:
                    obs.validate(now=self.clock(), max_age_s=self.config.max_observation_age_s, real=self.real)
                selected=prepare(proposals, obs, model=search_config) if prepare is not None else 0
                if type(selected) is not int or not 0 <= selected < len(proposals):
                    raise ValueError('Unknown selected push candidate')
                push, prediction=proposals[selected]
                self.events.emit('push_candidates_ranked', step=step, selected_rank=selected,
                    candidates=[dict(push=p.as_dict(), predicted_cost=r['predicted_cost']) for p,r in proposals],
                    evaluated_friction_rollouts=prediction['evaluated_friction_rollouts'])
                self.events.emit('push_planned', step=step, push=push.as_dict(), prediction=prediction,
                                 based_on={'session':obs.session_id, 'sequence':obs.sequence})
                self.stop.check()
                before=obs.pose; fit_allowed=True
                if self.interactive:
                    if self._pause_boundary():
                        invalidate_prepared(self.executor); pending_response=None
                        best=None; stagnant=0; need_stability=True; continue
                    latest=self._observe(obs.captured_at, obs)
                    if moved(obs.pose, latest.pose, self.control.policy):
                        invalidate_prepared(self.executor); self.control.epoch+=1
                        self.events.emit('push_plan_invalidated', reason='T_moved_during_planning',
                                         based_on_sequence=obs.sequence, latest_sequence=latest.sequence)
                        previous=latest; after=latest.captured_at
                        best=None; stagnant=0; need_stability=True; continue
                    # The selected contact feature is defined in the planning pose.
                    # Do not feed a shifted pose to the strict feature-identity fitter.
                    fit_allowed=not moved(obs.pose,latest.pose,
                        type(self.control.policy)(position_replan_m=.0001,yaw_replan_rad=.001))
                    previous=latest
                if prepare is None:
                    obs.validate(now=self.clock(), max_age_s=self.config.max_observation_age_s, real=self.real)
                self.control.state('executing', pose=before)
                self.executor.execute_push(push, obs)
                step+=1
                pending_response=(before, push, self.control.epoch) if fit_allowed else None
                if not fit_allowed:
                    self.events.emit('push_response_discarded',reason='pre_execution_motion_changed_contact_identity')
                after=self.clock()
                self.events.emit('push_command_finished', step=step-1, finished_at=after)
                self.control.state('observing', step=step)
        except BaseException:
            # No retry of unknown solver, collision, SDK, stale/session or execution errors.
            self.control.state('stopped_or_failed', step=step)
            raise
