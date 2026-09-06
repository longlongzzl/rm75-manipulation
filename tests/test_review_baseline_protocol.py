from tools.benchmark_world_model_baselines import candidates_for_budget, make_suite
import pytest


def test_equal_model_step_budget_has_different_candidate_counts():
    assert candidates_for_budget(192, 1) == 192
    assert candidates_for_budget(192, 3) == 64


@pytest.mark.parametrize('budget,horizon', [(0,1), (5,3), (10,0)])
def test_bad_budget_is_not_silently_rounded(budget, horizon):
    with pytest.raises(ValueError): candidates_for_budget(budget, horizon)


def test_frozen_synthetic_suite_is_reproducible():
    assert make_suite(4,17) == make_suite(4,17)
    assert make_suite(4,17) != make_suite(4,18)
