import pytest

from rm75_app.swm.native_body_mirror import _apply_kinematic_mode
from rm75_app.swm.scene import SceneInvalid


class Body:
    def __init__(self, initial):
        self._kinematic = initial
        self.auto_compute_mass = not initial
        self.writes = 0

    @property
    def kinematic(self):
        return self._kinematic

    @kinematic.setter
    def kinematic(self, value):
        self.writes += 1
        self._kinematic = value
        self.auto_compute_mass = False


@pytest.mark.parametrize('initial', [False, True])
def test_same_native_mode_does_not_reset_automatic_mass(initial):
    body = Body(initial)
    original_mass_mode = body.auto_compute_mass
    _apply_kinematic_mode(body, initial)
    assert body.writes == 0
    assert body.auto_compute_mass == original_mass_mode


def test_real_mode_change_is_applied_but_non_boolean_is_rejected():
    body = Body(False)
    _apply_kinematic_mode(body, True)
    assert body.kinematic is True and body.writes == 1
    with pytest.raises(SceneInvalid):
        _apply_kinematic_mode(body, 1)
    assert body.writes == 1
