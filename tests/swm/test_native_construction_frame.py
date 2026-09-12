"""Original record frame conversion must precede the native attachment call."""
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.swm import native_body_mirror
from rm75_app.swm.native_construction import NativeConstructionRecipe


@pytest.mark.parametrize('invalid', [False, True])
def test_record_frame_is_applied_before_native_builder_attaches(monkeypatch, invalid):
    monkeypatch.setitem(sys.modules, 'sapien', SimpleNamespace(
        physx=SimpleNamespace(PhysxMaterial=lambda *values: values)))
    monkeypatch.setattr(native_body_mirror, 'native_pose', lambda matrix: np.array(matrix, copy=True))
    calls = []
    class Builder:
        collision_records = []
        def build_physx_component(self):
            calls.append(self.collision_records[0].pose.copy())
            return 'fixture-attached-body'
    recipe = NativeConstructionRecipe.__new__(NativeConstructionRecipe)
    recipe.prototype = Builder()
    recipe.files = {}
    source = np.eye(4); source[:3, 3] = [.02, .03, .04]
    recipe.records = [(SimpleNamespace(scale=[1., 1., 1.]), source.copy(), (.6, .4, 0.))]
    actor_to_object = np.eye(4)
    actor_to_object[:2, :2] = [[0., -1.], [1., 0.]]
    actor_to_object[:3, 3] = [.1, -.2, .3]
    if invalid:
        actor_to_object[0, 1] = -2.
        with pytest.raises(ValueError):
            recipe.build_body(actor_to_object)
        assert calls == []
    else:
        assert recipe.build_body(actor_to_object) == 'fixture-attached-body'
        np.testing.assert_allclose(calls[0], np.linalg.inv(actor_to_object) @ source)
        np.testing.assert_array_equal(recipe.records[0][1], source)
        assert recipe.prototype.collision_records == []
