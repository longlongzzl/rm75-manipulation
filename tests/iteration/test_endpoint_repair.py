import copy
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.workcell.paired_endpoint_repair import complete_pairs,freeze_batch


def row(success,q=1):
    raw=SimpleNamespace(solution=np.full((1,1,7),q,dtype=float),success=np.array([[success]]),
                        position_error=np.zeros((1,1)),rotation_error=np.zeros((1,1)))
    return SimpleNamespace(success=success,goal_joint=np.full(7,q,dtype=float) if success else None,raw_result=raw,debug={})


def test_repair_missing_counterpart_only_same_goal():
    originals=[row(True,1),row(False),row(False),row(True,4)]
    calls=[]
    def query(seed,goal):
        calls.append((seed.copy(),goal));return row(True,goal+10)
    result,e=complete_pairs(originals,[np.zeros(7)]*4,[0,1,2,3],query,lambda q:True)
    assert len(calls)==2 and calls[0][1]==2 and calls[1][1]==1
    assert np.array_equal(calls[0][0],np.ones(7))
    assert np.array_equal(calls[1][0],np.full(7,4))
    assert all(r.success for r in result)
    assert e['newly_completed']==2 and not e['goals_modified'] and not e['full_chain_verified']
    assert originals[1].success is False and originals[2].success is False


@pytest.mark.parametrize('successes', [[False,False],[True,True]])
def test_no_query_when_both_same(successes):
    rows=[row(s) for s in successes]
    result,e=complete_pairs(rows,[0,0],[1,2],lambda *a:pytest.fail('must not query'),lambda q:True)
    assert not e['query_calls']


def test_failed_independent_collision_does_not_promote():
    result,e=complete_pairs([row(True),row(False)],[0,0],[1,2],lambda *a:row(True),lambda q:False)
    assert not result[1].success and not e['newly_completed']


def test_budget_seen_and_buffer_reuse():
    a=row(True,1);b=row(False);shared=a.raw_result;b.raw_result=shared
    def query(*a):
        shared.solution[:]=99
        return row(True,7)
    result,e=complete_pairs([a,b],[0,0],[1,2],query,lambda q:True)
    assert result[0].raw_result.solution[0,0,0]==1
    assert a.raw_result.solution[0,0,0]==99
    seen=set()
    _,e=complete_pairs([row(True),row(False)],[0,0],[1,2],query,lambda q:True,seen=seen)
    _,e2=complete_pairs([row(True),row(False)],[0,0],[1,2],lambda *a:pytest.fail(),lambda q:True,seen=seen)
    assert e['query_calls']==1 and e2['query_calls']==0


@pytest.mark.parametrize('limit,deadline',[(0,None),(5,1)])
def test_budget_never_starts_excess_query(limit,deadline):
    _,e=complete_pairs([row(True),row(False)],[0,0],[1,2],lambda *a:pytest.fail(),lambda q:True,
                      limit=limit,deadline=deadline,clock=lambda:2)
    assert e['query_calls']==0


def test_shape_mismatch_rejected():
    with pytest.raises(ValueError):complete_pairs([row(True)],[0],[1],lambda *a:None,lambda q:True)


def test_freeze_shared_raw_once():
    r=row(True);s=copy.copy(r)
    out=freeze_batch([r,s]);r.raw_result.solution[:]=8;r.goal_joint[:]=9
    assert out[0].raw_result is out[1].raw_result and out[0].raw_result is not r.raw_result
    assert out[0].goal_joint[0]==1 and out[0].raw_result.solution[0,0,0]==1
