import copy

import pytest

from rm75_app.swm.native_body_mirror import compare_native_state
from rm75_app.swm.scene import SceneInvalid


@pytest.mark.parametrize('changed', ['mass', 'material', 'shape', 'mask', 'missing'])
def test_native_body_readback_cannot_hide_physical_changes(changed):
    source = dict(mass=.5, shapes=[dict(kind='Box', material=.4, groups=[1, 1, 0, 0])])
    actual = copy.deepcopy(source)
    if changed == 'mass': actual['mass'] = 1.
    elif changed == 'material': actual['shapes'][0]['material'] = .1
    elif changed == 'shape': actual['shapes'].append(copy.deepcopy(actual['shapes'][0]))
    elif changed == 'mask': actual['shapes'][0]['groups'][0] = 0
    else: actual.pop('mass')
    with pytest.raises(SceneInvalid):
        compare_native_state(source, actual)


def test_native_body_readback_accepts_float_storage_precision_only():
    compare_native_state({'mass': .123456789}, {'mass': .123456791})
