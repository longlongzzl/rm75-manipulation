import numpy as np
import pytest
from rm75_app.swm.replay_initial_state import initial_velocities
from rm75_app.swm.scene import SceneInvalid


def test_observed_nonzero_velocities_are_not_replaced_by_rest():
    row=dict(fixed=False,measured=dict(linear_velocity=[.01,-.02,.03],angular_velocity=[.1,.2,-.3]))
    linear,angular,source=initial_velocities(row,required=True)
    np.testing.assert_array_equal(linear,[.01,-.02,.03])
    np.testing.assert_array_equal(angular,[.1,.2,-.3])
    assert source=='snapshot_velocity_readback'


@pytest.mark.parametrize('measured',[{},dict(linear_velocity=[0,0,0]),
    dict(linear_velocity=[0,0],angular_velocity=[0,0,0]),
    dict(linear_velocity=[float('nan'),0,0],angular_velocity=[0,0,0])])
def test_future_velocity_missing_partial_or_invalid_rejects(measured):
    with pytest.raises(SceneInvalid):initial_velocities(dict(fixed=False,measured=measured),required=True)


def test_fixed_object_cannot_carry_nonzero_velocity():
    with pytest.raises(SceneInvalid,match='Fixed object'):
        initial_velocities(dict(fixed=True,measured=dict(linear_velocity=[1,0,0],angular_velocity=[0,0,0])),required=True)


def test_legacy_absent_velocity_is_explicit_not_qualified_readback():
    _,_,source=initial_velocities(dict(fixed=False,measured={}),required=False)
    assert source=='legacy_unspecified_rest_assumption'
