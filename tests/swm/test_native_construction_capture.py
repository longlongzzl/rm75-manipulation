from types import SimpleNamespace

import pytest

from rm75_app.swm.native_construction import capture_actor_construction, recipe_for_actor
from rm75_app.swm.native_body_mirror import compare_native_state
from rm75_app.swm.scene import SceneInvalid


@pytest.mark.parametrize('values', [(2147483648,2147483649),(1,1.0),(True,1)])
def test_native_integer_and_boolean_identity_is_exact(values):
    with pytest.raises(SceneInvalid):
        compare_native_state({'identity':values[0]}, {'identity':values[1]})


def test_original_builder_capture_restores_method_on_failure():
    class Builder:
        def __init__(self):
            self.collision_records=[]
            self.collision_groups=[1,1,0,0]
            self.scene=object()
            self.visual_records=[object()]
        def build(self,name):
            return SimpleNamespace(_objs=[object()],name=name)
    original=Builder.build
    builder=Builder()
    with pytest.raises(RuntimeError,match='initialization failed'):
        with capture_actor_construction(Builder) as recipes:
            actor=builder.build('sample')
            recipe=recipe_for_actor(recipes,actor)
            assert recipe.prototype.scene is None
            assert recipe.prototype.visual_records == []
            assert builder.scene is not None and len(builder.visual_records)==1
            raise RuntimeError('initialization failed')
    assert Builder.build is original


def test_missing_native_recipe_cannot_fall_back_to_recooked_vertices():
    with pytest.raises(SceneInvalid,match='Exactly one'):
        recipe_for_actor({},SimpleNamespace(_objs=[object()]))
