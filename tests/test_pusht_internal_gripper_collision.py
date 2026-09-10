from copy import deepcopy
from itertools import combinations
from pathlib import Path

import pytest

from rm75_app.planning.backends.curobo2 import Curobo2BackendConfig, load_curobo2_robot_config
from rm75_app.planning.gripper_collision import (
    INTERNAL_GRIPPER_LINKS, ignore_gripper_internal_self_collision,
)
from rm75_app.pusht.motion import pusht_planner_options


CONFIG = Path(__file__).parents[1] / 'assets/curobo_rm75_config/rm75.yml'


def pairs(kinematics):
    return {frozenset((a, b)) for a, others in kinematics['self_collision_ignore'].items()
            for b in others}


def test_exclusion_covers_internal_pairs_without_removing_arm_or_world_geometry():
    original = load_curobo2_robot_config(CONFIG)['robot_cfg']['kinematics']
    changed = load_curobo2_robot_config(CONFIG, ignore_gripper_internal=True)['robot_cfg']['kinematics']
    internal = {frozenset(pair) for pair in combinations(INTERNAL_GRIPPER_LINKS, 2)}
    assert pairs(changed) == pairs(original) | internal
    for pair in (('gripper_Left_Support_Link', 'link_1'), ('gripper_Right_2_Link', 'base_link'),
                 ('gripper_Left_2_Link', 'link_6')):
        assert frozenset(pair) not in pairs(changed)
    for key in original:
        if key != 'self_collision_ignore':
            assert original[key] == changed[key]
    assert Curobo2BackendConfig().self_collision_check


def test_internal_exclusion_is_idempotent():
    kinematics = load_curobo2_robot_config(CONFIG)['robot_cfg']['kinematics']
    ignore_gripper_internal_self_collision(kinematics)
    before = deepcopy(kinematics)
    ignore_gripper_internal_self_collision(kinematics)
    assert kinematics == before


def test_task_default_is_scoped_to_pusht_and_allows_explicit_baseline_replay():
    assert not Curobo2BackendConfig().ignore_gripper_internal_self_collision
    assert pusht_planner_options()['ignore_gripper_internal_self_collision']
    options = {'ignore_gripper_internal_self_collision': False, 'position_tolerance': .001}
    assert pusht_planner_options(options) == options
    assert pusht_planner_options(options) is not options


def test_missing_gripper_model_fails_instead_of_broadly_disabling_collisions():
    with pytest.raises(ValueError, match='Incomplete gripper'):
        ignore_gripper_internal_self_collision({'collision_link_names': ['base_link', 'link_1']})
