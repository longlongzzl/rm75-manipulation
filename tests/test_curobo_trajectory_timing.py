from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.planning.backends.curobo2 import Curobo2Backend


class ArrayTensor:
    def __init__(self, values):
        self.values = np.asarray(values)

    @property
    def ndim(self):
        return self.values.ndim

    def __getitem__(self, index):
        return ArrayTensor(self.values[index])

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values


def test_interpolation_timing_survives_trimming_and_joint_reorder():
    planner = SimpleNamespace(trajopt_solver=SimpleNamespace(
        config=SimpleNamespace(interpolation_dt=0.025)))
    state = SimpleNamespace(position=ArrayTensor([[[1, 2], [3, 4], [5, 6]]]),
                            joint_names=("b", "a"))
    dt = Curobo2Backend._interpolation_dt(planner, state)
    trajectory = Curobo2Backend._trajectory_for_candidate(
        state, 0, ("a", "b"), end_tstep=2, dt=dt)
    np.testing.assert_array_equal(trajectory.positions, [[2, 1], [4, 3]])
    assert trajectory.dt == 0.025


def test_raw_knots_do_not_inherit_interpolation_timing():
    planner = SimpleNamespace(trajopt_solver=SimpleNamespace(
        config=SimpleNamespace(interpolation_dt=0.025)))
    assert Curobo2Backend._interpolation_dt(planner, None) is None
    assert Curobo2Backend._interpolation_dt(object(), object()) is None


@pytest.mark.parametrize("dt", [0, -0.1, float("nan"), float("inf")])
def test_invalid_solver_timing_is_rejected(dt):
    planner = SimpleNamespace(trajopt_solver=SimpleNamespace(
        config=SimpleNamespace(interpolation_dt=dt)))
    with pytest.raises(ValueError, match="interpolation_dt"):
        Curobo2Backend._interpolation_dt(planner, object())
