"""Software checks for post-capture execution admission, not native success."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.pusht.physics import PhysicsSession


def session(q):
    value=PhysicsSession.__new__(PhysicsSession)
    value.stop=SimpleNamespace(check=lambda:None)
    value.base=SimpleNamespace(read_q=lambda:np.asarray(q),active_program='previous')
    value.report={}
    return value


@pytest.mark.parametrize('q',[[np.nan]*7,[np.inf]*7,[0.]*6,[.000011]*7])
def test_invalid_or_drifted_start_rejects_without_changing_program(q):
    value=session(q)
    with pytest.raises(ValueError):
        value.check_stage_start(SimpleNamespace(initial=np.zeros(7),hold_stage='retreat'))
    assert value.base.active_program=='previous'


def test_native_start_read_is_not_cached():
    value=session(np.zeros(7));stage=SimpleNamespace(initial=np.zeros(7),hold_stage='approach')
    value.check_stage_start(stage)
    value.base.read_q=lambda:np.full(7,.001)
    with pytest.raises(ValueError,match='after boundary capture'):
        value.check_stage_start(stage)
    assert value.report['last_stage_start_check']['drift_rad']==.001


def test_original_start_limit_preserved():
    value=session(np.full(7,.000009))
    value.check_stage_start(SimpleNamespace(initial=np.zeros(7),hold_stage='push'))
    assert value.report['last_stage_start_check']['limit_rad']==1e-5
