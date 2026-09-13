import pytest
from rm75_app.pusht.retreat_contact import validate_contact_escape
from rm75_app.pusht.cartesian_ik import PushPathRejected

def contact(depth,**values):
    return dict(collision_type='world',world_object='pusht_target_0',
                robot_link='right_pad',penetration_m=depth,**values)

def test_existing_allowed_contact_must_clear():
    result=validate_contact_escape([[contact(.001)],[contact(.0005)],[]],{'right_pad'})
    assert result['initial_contact_pairs']==1 and result['final_contact_pairs']==0

@pytest.mark.parametrize('samples',[
    [[],[contact(.001)],[]],
    [[contact(.001)],[contact(.002)],[]],
    [[contact(.001)],[],[contact(.0001)],[]],
    [[contact(.001)],[contact(.0001)]],
    [[dict(contact(.001),world_object='table')],[]],
    [[dict(contact(.001),collision_type='self')],[]],
    [[dict(contact(.001),robot_link='joint_7')],[]],
])
def test_unsafe_retreat_rejects(samples):
    with pytest.raises(PushPathRejected):validate_contact_escape(samples,{'right_pad'})
