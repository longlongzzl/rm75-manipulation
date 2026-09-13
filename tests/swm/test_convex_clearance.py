from itertools import product
import numpy as np
import pytest
from rm75_app.planning.convex_clearance import certify_convex_clearance


def cube():
    return np.asarray(list(product((-.001, .001), repeat=3)))


def test_entire_convex_volume_clearance_preserves_all_explicit_margins():
    a = cube()
    b = a + [.02, 0, 0]
    result = certify_convex_clearance(a, b, margins={'global_pair': .005, 'self_pair': .008})
    assert result['sufficient_clearance_proven']
    assert result['conservative_gap_lower_bound_m'] <= .018
    assert result['residual_clearance_lower_bound_m'] == pytest.approx(.005)
    assert not result['execution_authorized'] and not result['policy_override_installed']
    assert not result['exact_minimum_distance']


@pytest.mark.parametrize('offset', [.002, .004, .001])
def test_touching_margin_and_overlap_never_certify(offset):
    result = certify_convex_clearance(cube(), cube()+[offset,0,0], margins={'required': .002})
    assert not result['sufficient_clearance_proven']
    assert not result['execution_authorized']


@pytest.mark.parametrize('margins', [{}, {'bad': -.001}, {'bad': float('nan')}])
def test_missing_or_invalid_clearance_rejected(margins):
    with pytest.raises(ValueError): certify_convex_clearance(cube(), cube(), margins=margins)
