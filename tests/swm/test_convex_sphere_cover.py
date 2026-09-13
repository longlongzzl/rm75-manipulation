import numpy as np
import pytest
from rm75_app.planning.convex_sphere_cover import cover_convex_volume, SphereCoverBudgetExceeded


def tetra():
    return np.asarray([[0,0,0],[.002,0,0],[0,.002,0],[0,0,.002]])


def test_complete_volume_coverage_and_float32_balls_not_deployable_flag():
    result = cover_convex_volume(tetra(), max_halfspace_excess_m=.001, max_spheres=256)
    assert result['completed'] and not result['deployable']
    proof = result['coverage']
    assert proof['full_convex_volume_covered'] and not proof['surface_sampling_only']
    assert proof['leaf_volume_sum_m3'] == pytest.approx(.002**3/6)
    assert proof['maximum_vertex_outside_ball_m'] <= 0
    assert proof['maximum_halfspace_excess_m'] <= .001
    for sphere in result['spheres']:
        assert np.array_equal(np.asarray(sphere['center'],dtype=np.float32), sphere['center'])
        assert float(np.float32(sphere['radius'])) == sphere['radius']


def test_budget_failure_never_emits_partial_deployable_cover():
    with pytest.raises(SphereCoverBudgetExceeded) as caught:
        cover_convex_volume(tetra(), max_halfspace_excess_m=1e-7, max_spheres=4)
    assert caught.value.evidence['deployable'] is False
    assert caught.value.evidence['completed'] is False


@pytest.mark.parametrize('value', [0., -.1, float('nan'), .02])
def test_invalid_tightness_rejected(value):
    with pytest.raises(ValueError): cover_convex_volume(tetra(), max_halfspace_excess_m=value)


def test_one_ball_can_certify_multiple_tetrahedra_without_surface_only_shortcut():
    from itertools import product
    cube = np.asarray(list(product((-.001, .001), repeat=3)))
    result = cover_convex_volume(cube, max_halfspace_excess_m=.001, max_spheres=16)
    assert result['sphere_count'] == 1
    assert result['proof_leaf_count'] == 12
    assert result['coverage']['leaf_volume_sum_m3'] == pytest.approx(.002**3)
    assert result['coverage']['full_convex_volume_covered']
