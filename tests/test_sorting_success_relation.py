from dataclasses import replace

import pytest

from test_sorting_scenario import _swap_request, _swap_scene
from rm75_app.scenarios.sorting import SortingPlanCompiler
from rm75_app.scenarios.sorting_io import sorting_request_as_dict, sorting_request_from_dict


def test_inside_is_explicit_and_survives_io_and_buffer_expansion():
    request = _swap_request(with_buffer=True)
    request = replace(request, targets=tuple(
        replace(target, success_relation="inside", support_object_id="b")
        if target.target_id == "target_for_a" else target for target in request.targets))
    restored = sorting_request_from_dict(sorting_request_as_dict(request))
    plan = SortingPlanCompiler().compile(restored, _swap_scene())
    buffer, inside, final = plan.atoms
    assert buffer.success.relation == "target_pose"
    assert inside.success.relation == inside.semantic_operator == "inside"
    assert inside.support_object_id == "b"
    assert final.success.relation == "target_pose"


def test_legacy_requests_remain_pose_based():
    payload = sorting_request_as_dict(_swap_request(with_buffer=True))
    for target in payload["targets"]:
        target.pop("success_relation")
    request = sorting_request_from_dict(payload)
    assert all(atom.success.relation == "target_pose"
               for atom in SortingPlanCompiler().compile(request, _swap_scene()).atoms)


@pytest.mark.parametrize("relation,support", [("inside", None), ("anything", "b")])
def test_invalid_success_contract_rejected(relation, support):
    target = _swap_request(with_buffer=True).targets[0]
    with pytest.raises(ValueError, match="success_relation"):
        replace(target, success_relation=relation, support_object_id=support)


@pytest.mark.parametrize("support,match", [("missing", "absent"), ("a", "itself")])
def test_inside_requires_existing_distinct_support(support, match):
    request = _swap_request(with_buffer=True)
    request = replace(request, targets=tuple(
        replace(target, success_relation="inside", support_object_id=support)
        if target.target_id == "target_for_a" else target for target in request.targets))
    with pytest.raises(ValueError, match=match):
        SortingPlanCompiler().compile(request, _swap_scene())
