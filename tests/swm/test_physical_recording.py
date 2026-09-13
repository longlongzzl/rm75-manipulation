from types import SimpleNamespace
import copy
import numpy as np
import pytest
from rm75_app.swm.native_simulation_clock import NativeSimulationClock
from rm75_app.swm.physical_recording import physical_recording_view
from rm75_app.swm.scene import SceneInvalid


def case():
    env=SimpleNamespace(scene=SimpleNamespace(px=SimpleNamespace(timestep=.01)),sim_freq=100,
        control_freq=20,_sim_steps_per_control=5,elapsed_steps=np.array([0]))
    clock=NativeSimulationClock(env)
    first=clock.read();env.elapsed_steps+=20;last=clock.read()
    pose=np.eye(4).tolist()
    a=dict(observation_domain='physics',robot=dict(simulation_clock=first),
        objects={'a':dict(measured=dict(captured_at=100.,simulation_clock=first))})
    b=copy.deepcopy(a);b['robot']['simulation_clock']=last
    b['objects']['a']['measured']=dict(captured_at=110.,simulation_clock=last)
    raw=dict(domain='physics',tool_captured_at=[99.,101.,111.],tool_simulation_clocks=[first,first,last],
        T_world_tcp=[copy.deepcopy(pose) for _ in range(3)],stages=['push']*3)
    return raw,a,b


def test_host_ten_seconds_maps_to_actual_one_physical_second_without_mutating_sources():
    raw,a,b=case();original=copy.deepcopy((raw,a,b))
    projected,before,after,evidence=physical_recording_view(raw,a,b,'a')
    assert projected['tool_captured_at']==[0.,1.]
    assert before['captured_at']==0. and after['captured_at']==1.
    assert evidence['duplicate_stationary_reads']==1
    assert (raw,a,b)==original


@pytest.mark.parametrize('fault',['epoch','coverage','object_clock','same_time_motion','missing_tool_clock'])
def test_incomplete_or_inconsistent_mapping_never_constructs_replay(fault):
    raw,a,b=case()
    if fault=='epoch': raw['tool_simulation_clocks'][-1]=dict(raw['tool_simulation_clocks'][-1],epoch='other')
    if fault=='coverage': raw['tool_captured_at'][-1]=109.
    if fault=='object_clock': b['objects']['a']['measured'].pop('simulation_clock')
    if fault=='same_time_motion':
        raw['T_world_tcp']=copy.deepcopy(raw['T_world_tcp']);raw['T_world_tcp'][1][0][3]=.01
    if fault=='missing_tool_clock': raw['tool_simulation_clocks'][1]=None
    with pytest.raises(SceneInvalid): physical_recording_view(raw,a,b,'a')
