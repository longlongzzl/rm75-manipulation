"""Software trajectory-boundary tests; not physical execution evidence."""
import numpy as np
import pytest
from rm75_app.pusht.physics_replay import TimedProgram,STAGES

def program():
    rows=[dict(stage=name,positions=[np.full(7,i).tolist(),np.full(7,i+1).tolist()],
               times=[0.,.07]) for i,name in enumerate(STAGES)]
    return TimedProgram(dict(complete_chain=True,validation_success=True,
        execute_real=False,hardware_connected=False,hardware_profile_qualified=False,
        stages=rows),lambda q:np.asarray(q))

def test_stage_holds_cannot_enter_successor():
    original=program();stages=original.stage_programs()
    assert tuple(p.hold_stage for p in stages)==STAGES
    for i,stage in enumerate(stages):
        name,q=stage.configuration(100.)
        assert name==STAGES[i]
        assert q==pytest.approx(np.full(7,i+1))
        assert stage.configuration(.035)[1]==pytest.approx(np.full(7,i+.5))
        assert stage.duration==.07

def test_stage_data_does_not_alias_original():
    original=program();stage=original.stage_programs()[0]
    stage.positions[:]=99
    assert original.rows[0][2][0]==pytest.approx(np.zeros(7))

def test_nonfinite_time_rejected():
    with pytest.raises(ValueError,match='Finite'):
        program().stage_programs()[0].configuration(float('nan'))
