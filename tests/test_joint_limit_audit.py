import numpy as np
import pytest

from rm75_app.validation.maniskill_gate import joint_limit_excess


def test_joint_limit_excess_preserves_both_sides_without_slack():
    np.testing.assert_allclose(joint_limit_excess(
        np.array([-1.1, 2.2, 0.0]), np.array([[-1, 1], [-2, 2], [0, 1]])), [0.1, 0.2, 0])


@pytest.mark.parametrize("q,limits", [([0], [[1, -1]]), ([float("nan")], [[0, 1]]),
                                      ([0, 1], [[0, 1]]), ([0], [[0, float("inf")]])])
def test_invalid_limit_audit_inputs_fail_closed(q, limits):
    with pytest.raises(ValueError):
        joint_limit_excess(np.array(q), np.array(limits))
