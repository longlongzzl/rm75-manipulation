"""Full recorder/binder contract with physics-shaped fixtures, not native evidence."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.feedback import MeasuredFeedbackRecorder
from rm75_app.swm.native_simulation_clock import NativeSimulationClock
from rm75_app.swm.identification import validate_transition
from rm75_app.swm.scene import SceneInvalid, digest
from rm75_app.swm.skills import SkillRequest
from .test_measurement_storage import recording
from .conftest import pose


def native_shaped_recording(rig):
    before,after,trace,receipt=recording(rig)
    env=SimpleNamespace(scene=SimpleNamespace(px=SimpleNamespace(timestep=.01)),sim_freq=100,
        control_freq=20,_sim_steps_per_control=5,elapsed_steps=np.array([0]))
    clock=NativeSimulationClock(env)
    clocks=[]
    for value in (0,10,20):
        env.elapsed_steps[:]=value;clocks.append(clock.read())
    for snapshot,stamp in ((before,clocks[0]),(after,clocks[-1])):
        snapshot['observation_domain']='physics'
        snapshot['robot']['simulation_clock']=copy.deepcopy(stamp)
        for item in snapshot['objects'].values():
            item['measured']['source']='native_primary_PhysX_readback'
            item['measured']['simulation_clock']=copy.deepcopy(stamp)
        snapshot['snapshot_id']=digest({k:v for k,v in snapshot.items() if k!='snapshot_id'})
    rows=[dict(source='measured_feedback',domain='physics',sensor_session='camera-session',
        calibration_id='extrinsics-v1',captured_at=t,T_world_tcp=p,stage=s,simulation_clock=c)
        for t,p,s,c in zip(trace['tool_captured_at'],trace['T_world_tcp'],trace['stages'],clocks)]
    return before,after,receipt,rows


def test_existing_recorder_requires_after_coverage_then_emits_physical_time(rig):
    before,after,receipt,rows=native_shaped_recording(rig)
    record=MeasuredFeedbackRecorder(domain='physics',sensor_session='camera-session',calibration_id='extrinsics-v1')
    record.append(rows[0])
    handle=record.begin_action(receipt.actual_action_id,receipt.started_at,support_id='table',
        initially_settled=True,settling_evidence='explicit fixture rest')
    record.append(rows[1]);receipt=record.finish_command(handle,receipt)
    request=SkillRequest('push','a',pose(.35,z=.03))
    with pytest.raises(SceneInvalid,match='cover both host exposures'):
        record.transition(request,receipt,before,after)
    assert receipt.actual_action_id in record._actions
    record.append(rows[2])
    result=record.transition(request,receipt,before,after)
    assert result['actual_action']['time_s']==[0.,.5,1.]
    assert result['object_time_s']==[0.,1.]
    assert result['initial_snapshot']==before
    assert result['physical_clock_evidence']['epoch']==rows[0]['simulation_clock']['epoch']
    assert not record._actions and not record._handles
    assert validate_transition(result)==result
    invalid=copy.deepcopy(result);invalid.pop('physical_clock_evidence')
    with pytest.raises(ValueError,match='projection evidence'): validate_transition(invalid)
