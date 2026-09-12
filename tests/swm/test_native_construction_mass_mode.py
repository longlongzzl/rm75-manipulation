import sys
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.swm.native_construction import NativeConstructionRecipe
from rm75_app.swm.scene import SceneInvalid


@pytest.mark.parametrize('measured', [True, False])
def test_measured_mass_mode_overrides_template_without_mutating_it(monkeypatch, measured):
    monkeypatch.setitem(sys.modules, 'sapien', SimpleNamespace())
    class Builder:
        _auto_inertial = not measured
        def build_physx_component(self):
            return self._auto_inertial
    recipe = NativeConstructionRecipe.__new__(NativeConstructionRecipe)
    recipe.prototype = Builder()
    recipe.records = []
    recipe.files = {}
    assert recipe.build_body(np.eye(4), auto_compute_mass=measured) is measured
    assert recipe.prototype._auto_inertial is not measured
    with pytest.raises(SceneInvalid, match='boolean'):
        recipe.build_body(np.eye(4), auto_compute_mass=1)
