"""Owned native control-boundary physical clock, separate from host wall time."""
from __future__ import annotations
import uuid
import numpy as np
from .native_bootstrap import _array
from .scene import ObservationUnavailable


class NativeSimulationClock:
    def __init__(self, env):
        self.env = env
        self.scene = env.scene
        self.physics = env.scene.px
        self.epoch = uuid.uuid4().hex
        self.period = self._period()
        self.origin = self._counter()
        self.last = self.origin

    def _period(self):
        dt = float(self.env.scene.px.timestep)
        sim, control = float(self.env.sim_freq), float(self.env.control_freq)
        substeps = self.env._sim_steps_per_control
        if (not np.isfinite([dt, sim, control, substeps]).all() or dt <= 0
                or sim <= 0 or control <= 0 or substeps < 1 or int(substeps) != substeps
                or sim/control != substeps):
            raise ObservationUnavailable('Original fixed native physical step period required')
        return dt, sim, control, int(substeps)

    def _counter(self):
        value = _array(self.env.elapsed_steps).reshape(-1)
        if (value.shape != (1,) or value.dtype.kind == 'b' or not np.isfinite(value).all()
                or not 0 <= value[0] <= 2**53 or int(value[0]) != value[0]):
            raise ObservationUnavailable('One finite integral native environment step counter required')
        return int(value[0])

    def read(self):
        if self.env.scene is not self.scene or self.env.scene.px is not self.physics or self._period() != self.period:
            raise ObservationUnavailable('Native physical clock scene or period changed')
        counter = self._counter()
        if counter < self.last:
            raise ObservationUnavailable('Native physical clock reset or regressed')
        self.last = counter
        dt, sim, control, substeps = self.period
        count = counter-self.origin
        return dict(source='original_env_elapsed_steps_and_native_PhysX_timestep',
            epoch=self.epoch, origin_control_counter=self.origin, native_control_counter=counter,
            elapsed_control_steps=count, elapsed_physics_substeps=count*substeps,
            physical_time_s=count*substeps*dt, native_physics_timestep_s=dt,
            physics_substeps_per_control=substeps, simulation_frequency_hz=sim,
            control_frequency_hz=control, scope='owned_original_env_control_boundaries',
            hardware_clock_qualified=False)
