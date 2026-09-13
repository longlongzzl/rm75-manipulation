"""Software counter contract only, no simulator or hardware creation."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.native_simulation_clock import NativeSimulationClock
from rm75_app.swm.scene import ObservationUnavailable


def env():
    return SimpleNamespace(scene=SimpleNamespace(px=SimpleNamespace(timestep=float(np.float32(.01)))),
        sim_freq=100,control_freq=20,_sim_steps_per_control=5,elapsed_steps=np.array([17]))


def test_physical_time_uses_native_dt_and_new_epoch_not_host_time():
    source = env()
    clock = NativeSimulationClock(source)
    first = clock.read()
    assert first['physical_time_s'] == 0
    source.elapsed_steps += 3
    last = clock.read()
    assert last['epoch'] == first['epoch']
    assert last['elapsed_physics_substeps'] == 15
    assert last['physical_time_s'] == 15 * float(np.float32(.01))
    assert last['native_control_counter'] == 20
    assert clock.read() == last
    assert not last['hardware_clock_qualified']


@pytest.mark.parametrize('change', ['reset','dt','frequency','scene','counter_shape','nan'])
def test_changed_clock_cannot_relabel_old_physical_time(change):
    source = env()
    clock = NativeSimulationClock(source)
    if change == 'reset': source.elapsed_steps[:] = 0
    if change == 'dt': source.scene.px.timestep = .02
    if change == 'frequency': source.control_freq = 10
    if change == 'scene': source.scene = SimpleNamespace(px=source.scene.px)
    if change == 'counter_shape': source.elapsed_steps = np.array([17,17])
    if change == 'nan': source.elapsed_steps = np.array([np.nan])
    with pytest.raises(ObservationUnavailable): clock.read()
