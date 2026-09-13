"""Request iterators own compiled-atom lifetime; never eagerly exhaust them."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.skills import SkillRequest
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.paired_grasp_place import PairedGraspPlace,atomic_groups


@pytest.mark.parametrize('success',[True,False])
def test_compiled_atom_remains_active_until_pair_finishes(success):
    active=[None];visited=[]
    def requests():
        for oid in ('a','b'):
            active[0]=oid
            yield SkillRequest('grasp',oid)
            yield SkillRequest('place',oid,target=np.eye(4))
            visited.append(oid)
        active[0]=None
    class Policy(PairedGraspPlace):
        def __init__(self):pass
        def run(self,runtime,grasp,place):
            assert active[0]==grasp.object_id==place.object_id
            assert grasp.skill=='grasp' and place.skill=='place'
            return [(grasp,SimpleNamespace(skill_verified=False)),(place,SimpleNamespace(skill_verified=success))]
    groups=atomic_groups(SimpleNamespace(grasp_place_policy=Policy()),requests())
    first=next(groups)
    assert first[0][0].object_id=='a' and active[0]=='a' and visited==[]
    remaining=list(groups)
    if success:assert len(remaining)==1 and visited==['a','b'] and active[0] is None
    else:assert remaining==[] and visited==[] and active[0]=='a'


def test_single_skill_iterator_is_lazy_too():
    active=[None]
    def requests():
        active[0]='a';yield SkillRequest('push','a',target=np.eye(4));active[0]=None
    def run(request):
        assert active[0]==request.object_id
        return SimpleNamespace(skill_verified=True)
    assert len(list(atomic_groups(SimpleNamespace(run=run),requests())))==1
    assert active[0] is None


def test_incomplete_pair_rejected_before_running_policy():
    class Policy(PairedGraspPlace):
        def __init__(self):pass
        def run(self,*args):raise AssertionError('Incomplete pair must not execute')
    with pytest.raises(SceneInvalid,match='Complete trusted'):
        list(atomic_groups(SimpleNamespace(grasp_place_policy=Policy()),[SkillRequest('grasp','a')]))
