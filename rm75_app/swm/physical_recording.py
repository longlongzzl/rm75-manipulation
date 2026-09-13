"""Project fully covered native recordings onto their measured physical epoch."""
import copy
import numpy as np
from .scene import SceneInvalid, pose_error


def _clock(clock):
    if not isinstance(clock, dict) or clock.get('source') != 'original_env_elapsed_steps_and_native_PhysX_timestep':
        raise SceneInvalid('Original native physical-clock evidence required')
    integers = ('origin_control_counter','native_control_counter','elapsed_control_steps',
                'elapsed_physics_substeps','physics_substeps_per_control')
    if any(type(clock.get(key)) is not int or clock[key] < 0 for key in integers):
        raise SceneInvalid('Exact integral native clock counters required')
    dt = clock.get('native_physics_timestep_s')
    sim, control = clock.get('simulation_frequency_hz'), clock.get('control_frequency_hz')
    if (not isinstance(clock.get('epoch'), str) or not clock['epoch']
            or not all(isinstance(x,(int,float)) and not isinstance(x,bool) and np.isfinite(x)
                       for x in (dt,sim,control,clock.get('physical_time_s')))
            or min(dt,sim,control) <= 0 or clock['physics_substeps_per_control'] < 1
            or sim/control != clock['physics_substeps_per_control']
            or clock['native_control_counter']-clock['origin_control_counter'] != clock['elapsed_control_steps']
            or clock['elapsed_control_steps']*clock['physics_substeps_per_control'] != clock['elapsed_physics_substeps']
            or clock['physical_time_s'] != clock['elapsed_physics_substeps']*dt):
        raise SceneInvalid('Inconsistent native physical-clock evidence')
    identity = (clock['epoch'],clock['origin_control_counter'],dt,sim,control,clock['physics_substeps_per_control'])
    return identity, float(clock['physical_time_s'])


def physical_recording_view(recording, initial, final, object_id):
    """Keep original snapshots/receipt host times intact; return replay copies."""
    raw = copy.deepcopy(recording)
    before = copy.deepcopy(initial['objects'][object_id]['measured'])
    after = copy.deepcopy(final['objects'][object_id]['measured'])
    if raw.get('domain') != 'physics' or any(s.get('observation_domain') != 'physics' for s in (initial,final)):
        raise SceneInvalid('Native physical mapping is simulation-only')
    host = np.asarray(raw['tool_captured_at'],dtype=float)
    clocks = raw.get('tool_simulation_clocks')
    if (host.ndim != 1 or len(host)<2 or not np.isfinite(host).all() or np.any(np.diff(host)<=0)
            or host[0]>before['captured_at'] or host[-1]<after['captured_at']
            or not isinstance(clocks,list) or len(clocks)!=len(host)
            or len(raw['T_world_tcp'])!=len(host) or len(raw['stages'])!=len(host)):
        raise SceneInvalid('Native feedback must cover both host exposures with per-sample clocks')
    identity,t0 = _clock(before.get('simulation_clock'))
    if _clock(initial['robot'].get('simulation_clock')) != (identity,t0):
        raise SceneInvalid('Initial object and robot physical times differ')
    end_identity,t1 = _clock(after.get('simulation_clock'))
    if end_identity != identity or _clock(final['robot'].get('simulation_clock')) != (identity,t1) or t1<=t0:
        raise SceneInvalid('Final object/robot must share a progressing physical epoch')
    times, poses, stages = [],[],[]
    for index,clock in enumerate(clocks):
        current,stamp = _clock(clock)
        if current != identity or (times and stamp<times[-1]):
            raise SceneInvalid('Mixed or regressed physical recording epoch')
        if times and stamp==times[-1]:
            p,r=pose_error(poses[-1],raw['T_world_tcp'][index])
            if p>1e-9 or r>1e-7:
                raise SceneInvalid('Different TCP poses at one native physical instant')
            continue
        times.append(stamp);poses.append(raw['T_world_tcp'][index]);stages.append(raw['stages'][index])
    if len(times)<2 or times[0]>t0 or times[-1]<t1:
        raise SceneInvalid('Native feedback does not bracket physical object observations')
    for row in raw.get('object_samples',[]):
        current,stamp=_clock(row.get('simulation_clock'))
        if current!=identity or not before['captured_at']<row['captured_at']<after['captured_at']:
            raise SceneInvalid('Interior observation does not belong to this physical epoch')
        row['captured_at']=stamp
    before['captured_at'],after['captured_at']=t0,t1
    raw['tool_captured_at'],raw['T_world_tcp'],raw['stages']=times,poses,stages
    return raw,before,after,dict(source='native_clock_projection_without_host_time_relabeling',
        epoch=identity[0],physical_start_s=t0,physical_end_s=t1,
        host_feedback_samples=len(host),physical_feedback_samples=len(times),
        duplicate_stationary_reads=len(host)-len(times),hardware_qualified=False)
