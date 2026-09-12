from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.swm.native_construction import capture_drive_construction


class Drive:
    def __init__(self):
        self.parent = object()
        self.entity = object()
        self.pose_in_parent = self.pose_in_child = SimpleNamespace(
            to_transformation_matrix=lambda: np.eye(4))
        self.applied = []

    def set_limit_x(self, low, high):
        self.applied.append((low, high))


class Scene:
    def create_drive(self):
        return Drive()


def test_actual_zero_lock_call_is_retained_in_order_and_methods_restored():
    original = Drive.set_limit_x
    create = Scene.create_drive
    with capture_drive_construction(Scene, Drive) as recipes:
        drive = Scene().create_drive()
        drive.set_limit_x(0., 0.)
        drive.set_limit_x(-1., 1.)
        assert recipes[id(drive)].calls == [
            ('set_limit_x', [0., 0.], {}), ('set_limit_x', [-1., 1.], {})]
        assert drive.applied == [(0., 0.), (-1., 1.)]
    assert Drive.set_limit_x is original
    assert Scene.create_drive is create


def test_native_capture_restores_methods_on_initialization_failure():
    original = Drive.set_limit_x
    with pytest.raises(RuntimeError, match='initialization'):
        with capture_drive_construction(Scene, Drive):
            raise RuntimeError('initialization failed')
    assert Drive.set_limit_x is original
