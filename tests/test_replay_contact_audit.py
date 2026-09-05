from types import SimpleNamespace as NS

from rm75_app.validation.maniskill_gate import ManiSkillJointAdapter


def contact(a, b, impulses):
    return NS(bodies=[NS(entity=NS(name=a)), NS(entity=NS(name=b))],
              points=[NS(impulse=value) for value in impulses])


def adapter(contacts):
    result = ManiSkillJointAdapter.__new__(ManiSkillJointAdapter)
    result.env = NS(unwrapped=NS(scene=NS(get_contacts=lambda: contacts)))
    result.robot_link_names = {"finger"}
    return result


def test_audit_keeps_object_contacts_without_changing_legacy_filter():
    audit = adapter([contact("finger", "ball", [[1, 0, 0]]),
                     contact("ball", "holder", [[0, 0, 1]])])
    assert len(audit.robot_contact_pairs()) == 1
    assert len(audit.robot_contact_pairs(include_nonrobot=True)) == 2


def test_opposing_contact_impulses_are_not_lost_in_net_zero():
    row = adapter([contact("finger", "ball", [[1, 0, 0], [-1, 0, 0]])]).robot_contact_pairs()[0]
    assert row["impulse_ns"] == 0
    assert row["point_impulse_norm_sum_ns"] == 2
    assert row["point_impulses_ns"] == [[1, 0, 0], [-1, 0, 0]]
