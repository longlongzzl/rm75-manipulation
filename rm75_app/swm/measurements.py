"""Join measured robot feedback to sparse, accepted visual checkpoints.

There is no mandatory realtime object tracker. Two object frames form an
endpoint-only experiment; extra accepted frames provide more informative data.
The tool recorder MUST cover the final object exposure time, including the
post-action wait. Requested joint/TCP trajectories never substitute for feedback.
"""
from __future__ import annotations
import copy
import numpy as np
from .scene import SceneInvalid,pose_error
from .identification import validate_transition
from .physics_replay import MeasuredTCPProgram


def bind_measured_transition(request,receipt,initial_snapshot,final_snapshot,*,recording):
    """Pure assembly after the after-skill checkpoint; no sensor/robot call here.

    recording is a trusted recorder snapshot, not browser/LLM input. tool_times
    are absolute capture times in the SAME calibrated host/simulation clock as
    SWM. Sources must provide a zero-motion t0 synchronized with the first frame.
    """
    raw=copy.deepcopy(recording)
    if (raw.get('schema')!='rm75_swm_action_recording_v1'
            or raw.get('actual_action_id')!=receipt.actual_action_id
            or raw.get('source')!='measured_feedback'):
        raise ValueError('Recording is not measured feedback for this execution')
    if request.skill!='push':
        # Articulated pull/rotate needs a constraint-aware replay backend; do not
        # fit a free-body push material model to a hinge, rail or held payload.
        return None
    if raw.get('intervened') is not False:return None
    oid=request.object_id;before=initial_snapshot['objects'][oid]['measured']
    after=final_snapshot['objects'][oid]['measured']
    if initial_snapshot['robot']['holding']!='empty' or final_snapshot['robot']['holding']!='empty':return None
    t0=before['captured_at'];t1=after['captured_at']
    if not t0<=receipt.started_at<=receipt.ended_at<t1:
        raise ValueError('Object/action clocks do not delimit the executed action')
    if (initial_snapshot['calibration_id']!=final_snapshot['calibration_id'] or
            initial_snapshot['sensor_session']!=final_snapshot['sensor_session']):
        raise SceneInvalid('Calibration or sensor session changed during the action')
    for key in ('domain','sensor_session','calibration_id'):
        expected=initial_snapshot['observation_domain'] if key=='domain' else initial_snapshot[key]
        if raw.get(key)!=expected:raise ValueError('Recorder and SWM clock/provenance mismatch')
    physical_clock_evidence = None
    if (initial_snapshot["robot"].get("simulation_clock") is not None
            or final_snapshot["robot"].get("simulation_clock") is not None
            or "tool_simulation_clocks" in raw):
        from .physical_recording import physical_recording_view
        raw,before,after,physical_clock_evidence = physical_recording_view(
            raw,initial_snapshot,final_snapshot,oid)
        t0,t1=before["captured_at"],after["captured_at"]
    tool_t=np.asarray(raw['tool_captured_at'],dtype=float)
    if (tool_t.ndim!=1 or len(tool_t)<2 or not np.isfinite(tool_t).all() or
            np.any(np.diff(tool_t)<=0) or tool_t[0]>t0 or tool_t[-1]<t1):
        raise ValueError('Measured tool feedback does not bracket BOTH object frames')
    program=MeasuredTCPProgram(dict(time_s=(tool_t-tool_t[0]).tolist(),
        T_world_tcp=raw['T_world_tcp'],stages=raw['stages']))
    timestamps=np.concatenate(([t0],tool_t[(tool_t>t0)&(tool_t<t1)],[t1]))
    sampled=[program.sample(float(t-tool_t[0])) for t in timestamps]
    p,r=pose_error(sampled[0][1],initial_snapshot['robot']['T_world_tcp'])
    if p>1e-9 or r>1e-7:
        raise ValueError('Checkpoint TCP was not time-aligned with the first visual exposure')
    interior=raw.get('object_samples',[])
    if not isinstance(interior,list):raise ValueError('Object samples must be a list')
    samples=[before]
    for row in interior:
        if not t0<row['captured_at']<t1:raise ValueError('Interior sample is outside this action interval')
        if row.get('id')!=oid or row.get('mesh_sha256')!=before['mesh_sha256']:
            raise ValueError('Interior observation belongs to another object/model')
        if row.get('source')!=before['source'] or row.get('accepted') is not True or row.get('tracking_state') in ('lost','recovering'):
            raise ValueError('Interior object measurement is not an accepted observation')
        if row.get('position_uncertainty_m',float('inf'))>.003 or row.get('rotation_uncertainty_rad',float('inf'))>.05:
            raise ValueError('Interior sample uncertainty exceeds identification budget')
        samples.append(row)
    samples.append(after)
    if raw.get('initially_settled') is not True:return None
    data=dict(schema='rm75_measured_transition_v1',domain=initial_snapshot['observation_domain'],
        object_id=oid,support_id=raw['support_id'],observation_source=before['source'],
        calibration_id=initial_snapshot['calibration_id'],sensor_session=initial_snapshot['sensor_session'],
        mesh_sha256=before['mesh_sha256'],initial_snapshot=copy.deepcopy(initial_snapshot),
        final_snapshot_id=final_snapshot['snapshot_id'],action_id=receipt.actual_action_id,
        initially_settled=True,settling_evidence=raw.get('settling_evidence'),intervened=False,holding_changed=False,
        actual_action=dict(source='measured_feedback',time_s=(timestamps-t0).tolist(),
            T_world_tcp=[a.tolist() for _,a in sampled],stages=[stage for stage,_ in sampled]),
        object_time_s=[float(row['captured_at']-t0) for row in samples],
        T_world_object=[row['T_world_object'] for row in samples],
        object_accepted=[row['accepted'] for row in samples],object_sequences=[row['sequence'] for row in samples])
    if physical_clock_evidence is not None:
        data["physical_clock_evidence"] = physical_clock_evidence
    return validate_transition(data)
