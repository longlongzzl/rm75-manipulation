import numpy as np
import pytest

from rm75_app.planning.gripper_collision import sphere_pair_penetration_m


@pytest.mark.parametrize('distance,expected',[(.035,.007),(.042,0.),(.05,-.008)])
def test_self_collision_reports_linear_metres_including_original_padding(distance,expected):
    a=[0,0,0,.02];b=[distance,0,0,.02]
    overlap=sphere_pair_penetration_m(a,b,.001,.001)
    assert overlap==pytest.approx(expected)
    squared_score=.042**2-distance**2
    if abs(expected)>1e-9:
        assert np.sign(overlap)==np.sign(squared_score)
        assert overlap!=pytest.approx(squared_score)


def test_nonfinite_sphere_is_not_reported_as_collision_free():
    with pytest.raises(ValueError):sphere_pair_penetration_m([np.nan,0,0,.02],[0,0,0,.02])
