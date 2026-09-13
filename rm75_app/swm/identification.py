"""Closed-loop physical parameter inference, distinct from response-gain fitting.

All hypotheses replay the SAME measured tool action from the SAME measured
initial state. Only material/density parameters differ. A posterior is a belief,
not a claim of unique true friction/mass; endpoint-only data cannot identify a
whole trajectory or generally resolve density under quasi-static pushing.
"""
from __future__ import annotations
import copy
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, asdict
import math
from typing import Callable
import numpy as np
from .scene import digest, transform, pose_error, positive, SceneInvalid


@dataclass(frozen=True)
class PhysicsParameters:
    static_friction: float
    dynamic_friction: float
    density_kg_m3: float

    def __post_init__(self):
        for key, value in vars(self).items(): positive(value, key)
        if self.dynamic_friction > self.static_friction:
            raise ValueError('Dynamic friction must not exceed static friction')
        if self.static_friction > 4 or not 1 <= self.density_kg_m3 <= 30000:
            raise ValueError('Physical parameter candidate outside supported bounds')

    def as_dict(self): return asdict(self)


@dataclass(frozen=True)
class ParameterBounds:
    static_friction: tuple = (.1, 1.2)
    dynamic_friction: tuple = (.05, 1.0)
    density_kg_m3: tuple = (100., 2000.)

    def __post_init__(self):
        for key, value in vars(self).items():
            if len(value)!=2: raise ValueError('Parameter range must be [min,max]')
            positive(value[0],key); positive(value[1],key)
            if value[0]>=value[1]: raise ValueError('Parameter range is empty')
        if self.dynamic_friction[0] >= self.static_friction[1]: raise ValueError('No valid friction pair')
        if self.static_friction[1]>4 or self.density_kg_m3[0]<1 or self.density_kg_m3[1]>30000:
            raise ValueError('Unsupported bounds')

    def contains(self, p):
        return all(a<=getattr(p,k)<=b for k,(a,b) in vars(self).items())


def sample_hypotheses(bounds, *, count=16, seed=0, posterior=None):
    """Sample around retained plausible models plus fixed prior exploration.

    Endpoint-only or uninformative data NEVER narrows density in this sampler.
    A 20% full-prior component and minimum kernel width prevent one lucky model
    being relabelled ground truth. All resampled candidates require new replay.
    """
    if type(count) is not int or not 2<=count<=128: raise ValueError('Bounded hypothesis count required')
    if type(seed) is not int or not 0<=seed<2**32: raise ValueError('Invalid seed')
    rng=np.random.default_rng(seed); result=[]; rows=[]; weights=[]
    if posterior and posterior.get('updated'):
        rows=posterior['particles'];weights=[r['weight'] for r in rows]
        if not rows or not np.isfinite(weights).all() or min(weights)<0 or sum(weights)<=0:
            raise ValueError('Malformed posterior weights')
        weights=np.asarray(weights)/sum(weights)
    for index in range(count):
        accepted=None
        for _ in range(500):
            local=bool(rows) and index >= max(1,math.ceil(count*.2))
            center=rows[int(rng.choice(len(rows),p=weights))]['parameters'] if local else None
            values={}
            for key,(lo,hi) in vars(bounds).items():
                # Dense trajectories can constrain effective dynamics, not prove a unique density.
                use_local=local and (key!='density_kg_m3' or posterior.get('density_informative') is True)
                if use_local:
                    values[key]=float(np.clip(rng.normal(center[key], .15*(hi-lo)),lo,hi))
                else:
                    values[key]=float(np.exp(rng.uniform(np.log(lo),np.log(hi)))) if key=='density_kg_m3' else float(rng.uniform(lo,hi))
            if values['dynamic_friction']<=values['static_friction']:
                accepted=PhysicsParameters(**values);break
        if accepted is None: raise ValueError('Could not sample a valid physical candidate')
        result.append(dict(id=f'h{seed:08x}_{index:03d}',parameters=accepted.as_dict()))
    return result


def _times(value, name, *, minimum=2):
    a=np.asarray(value,dtype=float)
    if a.ndim!=1 or len(a)<minimum or len(a)>100000 or not np.isfinite(a).all() or np.any(np.diff(a)<=0):
        raise ValueError(f'{name}: finite strictly increasing bounded sample times required')
    return a


def validate_transition(transition):
    data=copy.deepcopy(transition)
    if data.get('schema')!='rm75_measured_transition_v1': raise ValueError('Expected measured transition')
    if data.get('domain') not in ('real','physics','fixture'): raise ValueError('Missing measurement domain')
    allowed={'real':('foundationpose','rrtrack_foundationpose'), 'physics':('simulator_ground_truth','native_primary_PhysX_readback'), 'fixture':('fixture',)}
    if data.get('observation_source') not in allowed[data['domain']]:
        raise ValueError('Measured object poses need explicit valid observation provenance')
    if data.get('intervened') is not False or data.get('holding_changed') is not False:
        raise ValueError('External intervention/attachment changes are not physics calibration samples')
    if data.get('observation_source') == 'native_primary_PhysX_readback':
        from .physical_recording import _clock
        evidence = data.get('physical_clock_evidence')
        snapshot = data.get('initial_snapshot', {})
        measured = snapshot.get('objects', {}).get(data.get('object_id'), {}).get('measured', {})
        identity, start = _clock(measured.get('simulation_clock'))
        if (_clock(snapshot.get('robot', {}).get('simulation_clock')) != (identity, start)
                or not isinstance(evidence, dict)
                or evidence.get('source') != 'native_clock_projection_without_host_time_relabeling'
                or evidence.get('epoch') != identity[0] or evidence.get('physical_start_s') != start):
            raise ValueError('Native observations require consistent physical-clock projection evidence')
        end = evidence.get('physical_end_s')
        duration = data.get('object_time_s', [None])[-1]
        if (not isinstance(end, (int,float)) or not np.isfinite(end)
                or not isinstance(duration, (int,float)) or not np.isfinite(duration)
                or duration <= 0 or not np.isclose(end-start, duration, atol=1e-9, rtol=0)):
            raise ValueError('Native physical observation duration differs from projection')
    action=data.get('actual_action',{})
    if action.get('source') != 'measured_feedback':
        raise ValueError('Replay measured tool feedback, not a planned command trajectory')
    a_times=_times(action['time_s'],'actual tool action')
    o_times=_times(data['object_time_s'],'object observations')
    if a_times[0]!=0 or o_times[0]!=0 or o_times[-1] > a_times[-1]+1e-8:
        raise ValueError('Time alignment requires the same t0 and no extrapolation')
    accepted=data.get('object_accepted')
    if not isinstance(accepted,list) or len(accepted)!=len(o_times) or any(x is not True for x in accepted):
        raise ValueError('All object trace samples must pass the visual/physics observation gate')
    sequences=data.get('object_sequences')
    if not isinstance(sequences,list) or len(sequences)!=len(o_times) or any(type(x) is not int for x in sequences) or any(b<=a for a,b in zip(sequences,sequences[1:])):
        raise ValueError('Object trace sequences must be strictly increasing')
    if len(action['T_world_tcp']) != len(a_times) or len(data['T_world_object']) != len(o_times):
        raise ValueError('Time/pose sample count mismatch')
    for pose in action['T_world_tcp'] + data['T_world_object']: transform(pose)
    snap=data['initial_snapshot'];oid=data['object_id']
    if (not snap.get('valid') or snap['observation_domain']!=data['domain'] or oid not in snap['objects']
            or snap['calibration_id']!=data['calibration_id'] or snap['sensor_session']!=data['sensor_session']):
        raise ValueError('Transition/snapshot frame, session, domain or identity mismatch')
    canonical=dict(snap);sid=canonical.pop('snapshot_id',None)
    if digest(canonical)!=sid: raise ValueError('Initial snapshot was modified')
    obj=snap['objects'][oid];asset=snap['assets'][obj['asset_id']]
    if data['mesh_sha256']!=asset['mesh_sha256']: raise ValueError('Physical model identity mismatch')
    p,r=pose_error(obj['measured']['T_world_object'],data['T_world_object'][0])
    if p>1e-9 or r>1e-7: raise ValueError('Replay initial pose differs from the measured checkpoint')
    p,r=pose_error(snap['robot']['T_world_tcp'],action['T_world_tcp'][0])
    if p>1e-9 or r>1e-7: raise ValueError('Replay tool t0 differs from measured checkpoint')
    if data.get('initially_settled') is not True:
        raise ValueError('This replay adapter requires measured settled initial conditions')
    if not data.get('settling_evidence') or not data.get('action_id') or not data.get('support_id'):
        raise ValueError('Missing action/support/settling evidence')
    if data['support_id'] not in snap['objects'] or not snap['objects'][data['support_id']]['fixed']:
        raise ValueError('Calibration support is not registered fixed geometry')
    data['action_digest']=digest(action)
    data['observation_mode']='endpoint_only' if len(o_times)==2 else 'sampled_trajectory'
    data['transition_digest']=digest({k:v for k,v in data.items() if k!='transition_digest'})
    return data


class IsolatedReplayPool:
    """Bounded independent simulator instances. Real adapters are prohibited.

    factory(request) constructs a NEW private replay world for one hypothesis.
    No mutable world/executor is shared across workers. For GPU/SAPIEN native
    isolation prefer the process backend; threaded mode is for independent
    CPU-safe implementations. Default workers=1 respects existing machine limits.
    """
    def __init__(self, factory: Callable, *, workers=1):
        if type(workers) is not int or not 1<=workers<=4: raise ValueError('Replay concurrency is capped at four')
        self.factory=factory;self.workers=workers

    def run(self, transition, hypotheses, *, check=lambda:None):
        data=validate_transition(transition)
        ids=[r['id'] for r in hypotheses]
        if len(ids)!=len(set(ids)) or not 2<=len(ids)<=128: raise ValueError('Unique bounded hypotheses required')
        requests=[]
        for h in hypotheses:
            parameters=PhysicsParameters(**h['parameters'])
            requests.append(dict(hypothesis_id=h['id'],parameters=parameters.as_dict(),
                                 initial_snapshot=data['initial_snapshot'],actual_action=data['actual_action'],
                                 action_digest=data['action_digest'],transition_digest=data['transition_digest'],
                                 object_id=data['object_id'],support_id=data['support_id'],
                                 sample_times=data['object_time_s']))
        def one(request):
            check();world=None
            try:
                world=self.factory(copy.deepcopy(request))
                if world.domain not in ('physics','fixture') or data['domain']!='fixture' and world.domain!='physics':
                    raise PermissionError('Real/physical evidence cannot be calibrated by a surrogate fixture')
                result=world.replay()
                # Backend attests which actual initial/action/parameters were applied.
                for key in ('hypothesis_id','action_digest','transition_digest','parameters'):
                    if result.get(key)!=request[key]: raise ValueError(f'Replay changed {key}')
                if result.get('initial_snapshot_id')!=data['initial_snapshot']['snapshot_id']:
                    raise ValueError('Replay did not reset to the same observed state')
                result['engine_domain']=world.domain
                return result
            finally:
                if world is not None:world.close()
        # Errors are evidence, never silently replaced by a predicted successful path.
        outputs=[]
        with ThreadPoolExecutor(max_workers=self.workers) as pool:
            futures=[pool.submit(one,r) for r in requests]
            for request,future in zip(requests,futures):
                check()
                try:outputs.append(future.result())
                except Exception as exc:
                    outputs.append(dict(hypothesis_id=request['hypothesis_id'],valid=False,
                                        error_type=type(exc).__name__,reason='replay_failed'))
        return data,outputs


def infer_posterior(transition, hypotheses, rollouts, *, position_noise_m=.003,
                    rotation_noise_rad=.04):
    data=validate_transition(transition)
    positive(position_noise_m,'position_noise_m');positive(rotation_noise_rad,'rotation_noise_rad')
    by_id={r['hypothesis_id']:r for r in rollouts}
    if len(by_id)!=len(rollouts) or set(by_id)!={h['id'] for h in hypotheses}:
        raise ValueError('One explicit success/failure result per hypothesis is required')
    rows=[]
    for h in hypotheses:
        PhysicsParameters(**h['parameters'])
        r=by_id[h['id']];loss=None
        if r.get('valid') is True:
            if (r.get('action_digest')!=data['action_digest'] or r.get('transition_digest')!=data['transition_digest']
                    or r.get('parameters')!=h['parameters'] or r.get('initial_snapshot_id')!=data['initial_snapshot']['snapshot_id']):
                raise ValueError('Cannot mix different actions, starting scenes or parameters')
            if data['domain']!='fixture' and r.get('engine_domain')!='physics':
                raise ValueError('Physical evidence requires actual physical replays')
            if not np.array_equal(r.get('time_s'),data['object_time_s']):
                raise ValueError('Replays must be sampled at measured observation timestamps')
            if len(r.get('T_world_object',[]))!=len(data['object_time_s']): raise ValueError('Incomplete replay trajectory')
            errors=[pose_error(a,b) for a,b in zip(r['T_world_object'],data['T_world_object'])]
            # t0 is common and supplies no parameter information; do not dilute residuals with it.
            loss=float(np.mean([(p/position_noise_m)**2+(a/rotation_noise_rad)**2 for p,a in errors[1:]]))
        rows.append(dict(id=h['id'],parameters=h['parameters'],loss=loss,weight=0.))
    feasible=[r for r in rows if r['loss'] is not None and math.isfinite(r['loss'])]
    informative=len(feasible)>=2 and max(r['loss'] for r in feasible)-min(r['loss'] for r in feasible)>.1
    if feasible:
        losses=np.array([r['loss'] for r in feasible]);weights=np.exp(-.5*np.clip(losses-losses.min(),0,100))
        # Some prior mass remains on all feasible candidates, even when one fits better.
        weights=.8*weights/weights.sum()+.2/len(feasible)
        for row,w in zip(feasible,weights):row['weight']=float(w)
        best=min(feasible,key=lambda r:r['loss'])
    else:best=None
    return dict(schema='rm75_physics_posterior_v1',mesh_sha256=data['mesh_sha256'],
        object_id=data['object_id'],support_id=data['support_id'],domain=data['domain'],
        support_mesh_sha256=data['initial_snapshot']['assets'][data['initial_snapshot']['objects'][data['support_id']]['asset_id']]['mesh_sha256'],
        transition_digest=data['transition_digest'],actual_action_digest=data['action_digest'],
        observed_initial_snapshot_id=data['initial_snapshot']['snapshot_id'],
        observation_mode=data['observation_mode'],updated=informative,
        particles=rows, best_fit_id=None if best is None else best['id'],
        best_fit_parameters=None if best is None else best['parameters'],
        minimum_loss=None if best is None else best['loss'],
        density_informative=False, unique_parameters_identified=False,
        note='Effective material/density belief. No unique density claim; future actions must be replanned from a new checkpoint.')


def select_robust_candidate(candidates, posterior, *, min_weight=.01):
    """Candidate path must be feasible under every retained plausible hypothesis.

    An adapter supplies per-hypothesis audits. A fitting trajectory from an old
    initial state is never itself a candidate for the next real execution.
    """
    ids={p['id'] for p in posterior['particles'] if p['weight']>=min_weight}
    if not ids:raise ValueError('No retained physical hypotheses for robust planning')
    valid=[]
    for candidate in candidates:
        scores=candidate.get('hypothesis_scores',{})
        if set(scores)!=ids:continue
        if all(x.get('feasible') is True and np.isfinite(x.get('cost',float('inf'))) for x in scores.values()):
            valid.append((max(x['cost'] for x in scores.values()),candidate))
    if not valid:raise RuntimeError('No path is feasible across retained physical uncertainty')
    return copy.deepcopy(min(valid,key=lambda pair:pair[0])[1])


class AdaptivePhysicsManager:
    """Maintain/resample a per-instance physical belief after verified checkpoints.

    Callable from AtomicSkillRuntime.transition_observer. The adapter must supply
    real measured tool feedback and camera checkpoints; absent feedback is not
    replaced with the requested path. Fitting never executes a physical action.
    """
    def __init__(self, world, replay_pool, *, bounds=None, count=16, seed=42, emit=None, check=lambda:None, transition_builder=None):
        self.world=world;self.pool=replay_pool;self.bounds=bounds or ParameterBounds()
        self.count=count;self.seed=seed;self.emit=emit or (lambda **kw:None);self.check=check
        self._next={};self._seen=set();self._round=0
        self.transition_builder=transition_builder

    def on_skill(self, request, receipt, *, initial_snapshot, final_snapshot):
        """Bind post-action vision AFTER it is captured, rather than anticipating it.

        A trusted recorder must keep measured tool feedback until the post-skill
        frame arrives. It may assemble an endpoint-only transition; it may not
        fill missing feedback with the commanded path or a guessed stationary TCP.
        """
        if self.transition_builder is None:
            self.emit(kind='swm_physics_fit_skipped',reason='measured_transition_builder_not_installed')
            return None
        transition=self.transition_builder(request,receipt,initial_snapshot,final_snapshot)
        if transition is None:
            self.emit(kind='swm_physics_fit_skipped',reason='incomplete_or_intervened_measurements')
            return None
        if transition.get('action_id')!=receipt.actual_action_id or transition.get('object_id')!=request.object_id:
            raise SceneInvalid('Recorder returned evidence for a different atomic execution')
        return self(transition,initial_snapshot=initial_snapshot,final_snapshot=final_snapshot)

    def __call__(self, transition, *, initial_snapshot, final_snapshot):
        data=validate_transition(transition)
        if data['initial_snapshot']['snapshot_id']!=initial_snapshot['snapshot_id']:
            raise SceneInvalid('Identification sample starts from another planned/executed state')
        if data.get('final_snapshot_id')!=final_snapshot['snapshot_id'] or self.world.snapshot()['snapshot_id']!=final_snapshot['snapshot_id']:
            raise SceneInvalid('Identification sample is not bound to the latest post-skill checkpoint')
        if data['action_id'] in self._seen:raise SceneInvalid('Action already used for physics identification')
        oid=data['object_id'];actual=final_snapshot['objects'][oid]['measured']['T_world_object']
        p,r=pose_error(actual,data['T_world_object'][-1])
        if p>1e-9 or r>1e-7:raise SceneInvalid('Identification endpoint differs from the observed final object')
        # This free-body material experiment assumes a stationary support.
        # A moved fixture is a scene change, not a new friction measurement.
        support=data['support_id']
        sp,sr=pose_error(initial_snapshot['objects'][support]['measured']['T_world_object'],
                         final_snapshot['objects'][support]['measured']['T_world_object'])
        if sp>.001 or sr>.01:raise SceneInvalid('Support moved during the identification interval')
        before_revision=self.world.physics_revision
        prior=final_snapshot['physics'].get(oid)
        if prior and prior.get('support_id')!=data['support_id']:prior=None
        bank=sample_hypotheses(self.bounds,count=self.count,seed=(self.seed+self._round)%(2**32),posterior=prior)
        self._seen.add(data['action_id']);self._round+=1
        checked,rollouts=self.pool.run(data,bank,check=self.check)
        posterior=infer_posterior(data,bank,rollouts)
        if self.world.snapshot()['snapshot_id']!=final_snapshot['snapshot_id']:
            raise SceneInvalid('Scene changed during replay; no parameter update is committed')
        if posterior['updated']:
            self.world.update_physics(oid,posterior,expected_physics_revision=before_revision)
        self._next[oid]=sample_hypotheses(self.bounds,count=self.count,seed=(self.seed+self._round)%(2**32),
                                        posterior=posterior if posterior['updated'] else prior)
        self.emit(kind='swm_physics_identification',object_id=oid,action_id=data['action_id'],
                  hypotheses=len(bank),valid_rollouts=sum(r.get('valid') is True for r in rollouts),
                  updated=posterior['updated'],observation_mode=posterior['observation_mode'],
                  best_fit_id=posterior['best_fit_id'],unique_parameters_identified=False)
        return dict(posterior=posterior,next_hypotheses=copy.deepcopy(self._next[oid]),
                    replan_from_current_measurement=True)
