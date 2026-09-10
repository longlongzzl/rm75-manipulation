import copy
import pytest
from rm75_app.pusht.scenarios import sample_scenarios,configured_model
from rm75_app.pusht.model import valid_pose,reached


@pytest.mark.parametrize('seed',list(range(20)))
def test_random_cases_valid_unique_reproducible(profile,seed):
    suite=sample_scenarios(profile,seed=seed,count=6)
    assert suite==sample_scenarios(profile,seed=seed,count=6)
    cfg=configured_model(profile)
    assert len(suite['cases'])==6
    assert len({x['state_digest'] for x in suite['cases']})==6
    assert {x['mode'] for x in suite['cases']}=={'translation','rotation','mixed'}
    for case in suite['cases']:
        assert valid_pose(case['initial_pose'],cfg) and valid_pose(case['goal_pose'],cfg)
        assert not reached(case['initial_pose'],case['goal_pose'],cfg)
        assert case['robot_planning_verified'] is False


def test_shape_variants_drop_original_fit(profile):
    profile['pusht']['model']['response_fits']=[{'feature':0,'gains':[1,0,1],'samples':1,'sum_xy':[0,0,0],'sum_xx':[0,0,0]}]
    cfg=configured_model(profile,geometry_id='wide')
    assert cfg.bar_width_m==.12 and not cfg.response_fits
    assert configured_model(profile).response_fits


@pytest.mark.parametrize('key',['position_tolerance_m','obstacles','workspace','response_fits','speed_mps'])
def test_variant_not_a_safety_backdoor(profile,key):
    profile['pusht']['physics']['geometry_variants']['bad']={key:0}
    with pytest.raises(ValueError):configured_model(profile,geometry_id='bad')


@pytest.mark.parametrize('seed,count',[(-1,1),(True,2),(1,65),(0,0)])
def test_random_input_bounds(profile,seed,count):
    with pytest.raises(ValueError):sample_scenarios(profile,seed=seed,count=count)
