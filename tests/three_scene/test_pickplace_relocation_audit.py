from copy import deepcopy

import pytest

from tools.audit_pickplace_relocation import compare_scene, margin_summary


def saved_row(overlap=.01):
    return dict(event='pickplace_failed_lift_ik_diagnostic', source='gluestick',
        step_id='joint_start_lift', collision_model_sha256='saved-model-hash',
        diagnostic_complete=True, state_unchanged=True, attached=True, native_success=False,
        start={'fk': {'position': [1., 2., .016]}}, goal_pose={'position': [1., 2., .096]},
        nominal_goal_payload_base=dict(geometric_necessary_condition_only=True,
            physical_geometry_qualified=False, contacts=[dict(link_a='attached_object',
                link_b='base_link', sphere_i=19, sphere_j=0, overlap_m=overlap, pair_ignored=False)]))


@pytest.mark.parametrize('overlap,gap,spheres_overlap', [(.01, .03, False), (.04, 0., False), (.05, -.01, True)])
def test_margin_decomposition_never_promotes_failed_task(overlap, gap, spheres_overlap):
    raw = saved_row(overlap)
    before = deepcopy(raw)
    row = margin_summary(raw, {'base_link': .04, 'attached_object': 0.})
    contact, = row['contacts']
    assert contact['gap_without_link_self_buffers_m'] == pytest.approx(gap)
    assert contact['underlying_spheres_still_overlap'] is spheres_overlap
    assert contact['original_query_remains_failed'] is True
    assert row['original_native_success'] is False and row['safe_to_execute'] is False
    assert row['physical_geometry_qualified'] is False and row['new_collision_query'] is False
    assert row['recorded_goal_minus_start_fk_z_m'] == pytest.approx(.08)
    assert row['recorded_goal_minus_start_fk_xy_norm_m'] == 0.
    assert raw == before


@pytest.mark.parametrize('field,value', [('event', 'unrelated'), ('diagnostic_complete', False),
    ('state_unchanged', False), ('native_success', True), ('attached', False)])
def test_incomplete_or_unrelated_gpu_record_not_accepted(field, value):
    raw = saved_row()
    raw[field] = value
    with pytest.raises(ValueError):
        margin_summary(raw, {'base_link': .04, 'attached_object': 0.})


@pytest.mark.parametrize('case', ['no_contacts', 'ignored', 'other_link', 'nan_overlap', 'qualified', 'bad_position'])
def test_no_empty_or_unqualified_geometry_can_pass(case):
    raw = saved_row()
    nominal = raw['nominal_goal_payload_base']
    if case == 'no_contacts':
        nominal['contacts'] = []
    elif case == 'ignored':
        nominal['contacts'][0]['pair_ignored'] = True
    elif case == 'other_link':
        nominal['contacts'][0]['link_b'] = 'gripper_base_link'
    elif case == 'nan_overlap':
        nominal['contacts'][0]['overlap_m'] = float('nan')
    elif case == 'qualified':
        nominal['physical_geometry_qualified'] = True
    else:
        raw['goal_pose']['position'] = [1, 2, float('inf')]
    with pytest.raises(ValueError):
        margin_summary(raw, {'base_link': .04, 'attached_object': 0.})


@pytest.mark.parametrize('value', [-.01, float('nan'), float('inf')])
def test_invalid_buffer_rejected(value):
    with pytest.raises(ValueError):
        margin_summary(saved_row(), {'base_link': value, 'attached_object': 0.})


def test_missing_buffer_not_silently_zero():
    with pytest.raises(KeyError):
        margin_summary(saved_row(), {'base_link': .04})


def test_only_relocated_provenance_is_distinguished_from_scene_data():
    old = {'objects': {'gluestick': {'T_world_obj': 'fixed', 'placed': False}},
           'generated_metadata': {'base_scene_file': 'old.json', 'seed': 42}}
    new = deepcopy(old)
    new['generated_metadata']['base_scene_file'] = 'new.json'
    row = compare_scene(old, new)
    assert row['provenance_path_only_difference'] and row['objects_equal']
    assert row['json_equal'] is False and row['changed_object_names'] == []
    new['generated_metadata']['seed'] = 43
    assert not compare_scene(old, new)['provenance_path_only_difference']
    new['objects']['gluestick']['placed'] = True
    changed = compare_scene(old, new)
    assert not changed['objects_equal'] and changed['changed_object_names'] == ['gluestick']


@pytest.mark.parametrize('empty', [{}, {'objects': {}}, {'objects': []}])
def test_empty_scene_never_counts_as_equivalent(empty):
    with pytest.raises(ValueError):
        compare_scene(empty, deepcopy(empty))
