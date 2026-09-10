from types import SimpleNamespace as NS

import pytest
import torch

from rm75_app.planning.backends.curobo2 import Curobo2Backend


@pytest.mark.parametrize('shared', [False, True])
@pytest.mark.parametrize('fail', [False, True])
def test_suspension_restores_exact_dynamic_geometry_and_preexisting_disabled_spheres(shared, fail):
    original=torch.tensor([[[.1,.2,.3,.01],[.4,.5,.6,-100.]]])
    first=original.clone();second=first if shared else original.clone()
    def kinematics(spheres):
        return NS(config=NS(kinematics_config=NS(link_spheres=spheres)))
    planner=NS(kinematics=kinematics(first),ik_solver=NS(kinematics=kinematics(second)))
    backend=Curobo2Backend.__new__(Curobo2Backend)
    backend._ensure_planner=lambda:planner
    try:
        with backend.suspend_collision_checks():
            for spheres in (first,second):
                assert torch.all(spheres[...,3]==-100.)
                torch.testing.assert_close(spheres[...,:3],original[...,:3])
            if fail:
                raise RuntimeError('IK failure')
    except RuntimeError:
        assert fail
    for spheres in (first,second):
        torch.testing.assert_close(spheres,original,rtol=0,atol=0)
