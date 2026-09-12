"""Exact extraction contract; native CUDA timing is tested separately."""
import numpy as np
import pytest
from rm75_app.planning.backends.curobo2 import Curobo2Backend


class Tensor:
    def __init__(self, value, calls):
        self.value = np.asarray(value)
        self.ndim = self.value.ndim
        self.calls = calls

    def __gt__(self, other):
        return Tensor(self.value > other, self.calls)

    def nonzero(self):
        self.calls.append('nonzero')
        return Tensor(np.argwhere(self.value), self.calls)

    def detach(self):
        return self

    def cpu(self):
        self.calls.append('cpu')
        return self

    def tolist(self):
        return self.value.tolist()


@pytest.mark.parametrize('shape', [(1, 1), (64, 128), (119, 256)])
def test_batched_indices_preserve_every_positive_contact_and_original_order(shape):
    values = np.random.default_rng(713).normal(size=shape)
    values.flat[0] = 0.
    expected = [[row, col] for row in range(shape[0]) for col in range(shape[1])
                if values[row, col] > 0]
    calls = []
    assert Curobo2Backend._positive_contact_indices(Tensor(values, calls)) == expected
    assert calls == ['nonzero', 'cpu']


def test_collision_free_batch_still_uses_only_one_transfer():
    calls = []
    assert Curobo2Backend._positive_contact_indices(Tensor(np.zeros((64, 128)), calls)) == []
    assert calls == ['nonzero', 'cpu']


def test_wrong_rank_is_rejected():
    with pytest.raises(ValueError, match='batch-by-feature'):
        Curobo2Backend._positive_contact_indices(Tensor(np.zeros((2, 1, 3)), []))
