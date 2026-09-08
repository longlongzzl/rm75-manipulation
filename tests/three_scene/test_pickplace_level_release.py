from types import SimpleNamespace as NS
import numpy as np
import pytest
from rm75_app.workcell.pickplace_level_release import filter_candidates, install, level_pose


def pose(q=(0, 1, 0, 0)):
    return NS(q=np.array(q, dtype=float))


def candidate(pre=None, release=None):
    return dict(pose=pre or pose(), place_pose=release or pose(), label='original')


def test_only_original_level_downward_poses_survive_no_tilted_fallback():
    good=candidate();bad=candidate(release=pose((.24986,-.837516,-.481226,-.067506)))
    result=filter_candidates([bad,good],np.asarray)
    assert len(result)==1 and result[0] is good
    assert filter_candidates([bad],np.asarray)==[]
    assert bad['place_pose'].q[0]==.24986


@pytest.mark.parametrize('key',['pose','place_pose','pre_place_pose','hover_pose','release_pose'])
def test_every_pre_and_release_pose_must_be_level(key):
    value=candidate();value[key]=pose((.24986,-.837516,-.481226,-.067506))
    assert filter_candidates([value],np.asarray)==[]


@pytest.mark.parametrize('q',[(1,0,0,0),(0,0,0,0),(float('nan'),1,0,0),(1,0,0)])
def test_invalid_or_upward_pose_is_not_a_level_downward_release(q):
    assert not level_pose(pose(q),np.asarray)


def test_level_yaw_variants_are_preserved():
    for yaw in np.linspace(-np.pi,np.pi,15):
        assert level_pose(pose((0,np.cos(yaw/2),np.sin(yaw/2),0)),np.asarray)


def test_adapter_is_tennis_only_preserves_original_calls_and_restores_functions():
    calls=[];rows=[];good=candidate();bad=candidate(release=pose((1,0,0,0)))
    def generator(demo,args,*,override=None):
        calls.append((demo,args,override));return [bad,good]
    direct=NS(_build_direct_pre_place_candidates=generator,_build_direct_place_candidates=generator,
              _current_source_object_name=lambda args:args.source,targeted=NS(base=NS(flatten_np=np.asarray)))
    close=install(direct,rows.append)
    args=NS(source='tennis')
    assert direct._build_direct_place_candidates('demo',args,override=3)==[good]
    assert calls==[('demo',args,3)] and rows[0]['original_count']==2
    for source in ('gluestick','hongshupian','front_wall','bi'):
        assert len(direct._build_direct_pre_place_candidates(None,NS(source=source)))==2
    close();assert direct._build_direct_place_candidates is generator
