"""Link-body fixtures do not qualify physics stepping or controller replay."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.native_robot_mirror import link_body_policy, align_link_body_policy
from rm75_app.swm.native_body_mirror import compare_native_state
from rm75_app.swm.scene import SceneInvalid


class Pose:
    def __init__(self, matrix): self.matrix = np.asarray(matrix).copy()
    def to_transformation_matrix(self): return self.matrix.copy()


def body():
    return SimpleNamespace(mass=.2, inertia=np.array([.001,.002,.003]),
        cmass_local_pose=Pose(np.eye(4)), auto_compute_mass=False, disable_gravity=False,
        linear_damping=.01, angular_damping=.02)


@pytest.fixture(autouse=True)
def native_pose_fixture(monkeypatch):
    monkeypatch.setattr('rm75_app.swm.native_body_mirror.native_pose', Pose)


def test_manual_inertia_and_gravity_copy_never_changes_primary():
    source, private = {'link':body()}, {'link':body()}
    source['link'].disable_gravity = True
    source['link'].mass = .5
    source['link'].cmass_local_pose.matrix[0,3] = .01
    before = link_body_policy(source['link'])
    expected = align_link_body_policy(source, private)
    assert expected['link'] == link_body_policy(source['link']) == before
    compare_native_state(before, link_body_policy(private['link']))


@pytest.mark.parametrize('field,value', [('mass',float('nan')), ('mass',-1.),
    ('inertia',np.array([1.,-1.,1.])), ('inertia',np.array([1.,2.])),
    ('inertia',np.array([1.,float('inf'),1.])), ('linear_damping',-1.)])
def test_invalid_body_values_reject_before_writes(field,value):
    src,dst=body(),body()
    before=link_body_policy(dst)
    setattr(src,field,value)
    with pytest.raises(SceneInvalid): align_link_body_policy({'link':src},{'link':dst})
    assert link_body_policy(dst)==before


def test_automatic_mass_mismatch_cannot_be_replaced_with_manual_values():
    src,dst=body(),body()
    src.auto_compute_mass=True
    before=link_body_policy(dst)
    with pytest.raises(SceneInvalid,match='automatic_mass'):
        align_link_body_policy({'link':src},{'link':dst})
    assert link_body_policy(dst)==before


def test_matching_automatic_mass_policy_is_preserved():
    src,dst=body(),body()
    src.auto_compute_mass=dst.auto_compute_mass=True
    align_link_body_policy({'link':src},{'link':dst})
    assert dst.auto_compute_mass


@pytest.mark.parametrize('bad',['alias','missing','duplicate'])
def test_link_inventory_requires_independent_unique_bodies(bad):
    src={'a':body(),'b':body()};dst={'a':body(),'b':body()}
    if bad=='alias': dst['a']=src['b']
    if bad=='missing': dst.pop('b')
    if bad=='duplicate': dst['b']=dst['a']
    with pytest.raises(SceneInvalid,match='inventory'): align_link_body_policy(src,dst)


def test_native_body_drift_is_detected_not_hidden_by_cache():
    src,dst={'a':body()},{'a':body()}
    expected=align_link_body_policy(src,dst)
    dst['a'].mass=.8
    with pytest.raises(SceneInvalid):
        compare_native_state(expected,{'a':link_body_policy(dst['a'])})
