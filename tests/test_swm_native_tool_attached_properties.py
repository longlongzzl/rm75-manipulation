import pytest

from rm75_app.swm.native_tool_replay import apply_attached_shape_properties
from rm75_app.swm.scene import SceneInvalid


class AttachedShape:
    contact_offset=0.01

    @property
    def density(self):return 1000.0

    @density.setter
    def density(self,value):raise RuntimeError('attached density is immutable')


def test_original_density_is_read_not_written():
    shape=AttachedShape()
    apply_attached_shape_properties(shape,dict(density=1000.0,contact_offset=0.02))
    assert shape.density==1000.0
    assert shape.contact_offset==0.02


def test_changed_construction_density_rejects():
    with pytest.raises(SceneInvalid,match='tool_shape.density'):
        apply_attached_shape_properties(AttachedShape(),dict(density=2000.0))
