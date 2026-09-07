from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import trimesh

from tools.audit_jimu_triangle_assets import (
    ROOT, PRIMARY, FALLBACK, builder_report, collider_report,
    mesh_report, native_helpers, selection_equivalent,
)

SNAPSHOT = ROOT / 'rm75_app/_vendor/working_snapshot'


def prism(extents, tip_up=True):
    w, t, h = extents
    vertices = [[x, y, z] for y in (-t / 2, t / 2)
                for x, z in ((-w / 2, -h / 2), (w / 2, -h / 2), (0, h / 2))]
    vertices = np.asarray(vertices)
    if not tip_up:
        vertices[:, 2] *= -1
    faces = [[0, 2, 1], [3, 4, 5], [0, 1, 4], [0, 4, 3],
             [1, 2, 5], [1, 5, 4], [2, 0, 3], [2, 3, 5]]
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)


@pytest.fixture
def assets(tmp_path):
    fallback = tmp_path / FALLBACK
    fallback.parent.mkdir(parents=True)
    fallback.write_bytes(prism([.54, .07, 1.], tip_up=False).export(file_type='glb'))
    return tmp_path


def primary(assets):
    path = assets / PRIMARY
    path.parent.mkdir(parents=True)
    path.write_bytes(prism([.074, .0065, .135]).export(file_type='glb'))
    return path


def pieces():
    parent = dict(id='parent', role='right_second_wall', type='square', locked=False,
        center=[0., .11425, 0.], u=[1., 0., 0.], n=[0., 0., -1.], v=[0., 1., 0.])
    child = dict(id='roof', role='right_roof_triangle', type='triangle', locked=False,
        center=[0., .21875, 0.], u=[-1., 0., 0.], n=[0., 0., 1.], v=[0., 1., 0.],
        parentId='parent', parentEdge='top', childAttachEdge='bottom',
        parentRelativeTransform=[[-1., 0., 0., 0.], [0., -1., 0., 0.],
                                 [0., 0., 1., .1045], [0., 0., 0., 1.]])
    return [parent, child]


def test_native_primary_exists_changes_spec_geometry_and_tip(assets):
    primary(assets)
    context = native_helpers(SNAPSHOT, assets)
    row = mesh_report(context, assets)
    assert row['selected_relative_path'] == str(PRIMARY)
    assert row['native_spec_uses_primary'] is True
    assert row['native_sim_scale'] == 1.
    assert row['native_extents_m'] == pytest.approx([.074, .0065, .135])
    assert row['mesh_tip_local_z_sign'] == 1
    assert row['native_standard_tip_dot_parent_up'] == 1.
    assert not row['physical_geometry_qualified']


def test_native_missing_primary_falls_back_without_geometry_equivalence(assets):
    context = native_helpers(SNAPSHOT, assets)
    row = mesh_report(context, assets)
    assert row['selected_relative_path'] == str(FALLBACK)
    assert row['native_spec_uses_primary'] is False
    assert row['native_sim_scale'] == pytest.approx(.12)
    assert row['native_extents_m'] == pytest.approx([.0648, .0084, .12])
    assert row['mesh_tip_local_z_sign'] == -1
    # Standard target code corrects the tip; direct builder axes do not.
    assert row['native_standard_tip_dot_parent_up'] == 1.


@pytest.mark.parametrize('with_primary,tip,gap', [(True, 1., .004), (False, -1., .0115)])
def test_exact_native_builder_frame_preserved_and_mesh_difference_exposed(assets, with_primary, tip, gap):
    if with_primary:
        primary(assets)
    context = native_helpers(SNAPSHOT, assets)
    model = mesh_report(context, assets)
    design = pieces()
    before = deepcopy(design)
    row, = builder_report(context, design, model, wall_height_m=.074, roof_increment_m=.004)
    assert design == before
    assert row['parent_relative_closure_max_abs'] < 1e-7
    assert row['mesh_tip_dot_builder_up'] == tip
    assert row['nominal_bbox_edge_gap_with_original_layer_extra_m'] == pytest.approx(gap, abs=1e-7)
    assert row['task_success'] is None and row['necessary_geometry_only']


def test_explicit_parent_transform_disagreement_is_recorded_not_silently_rewritten(assets):
    context = native_helpers(SNAPSHOT, assets)
    model = mesh_report(context, assets)
    design = pieces()
    design[1]['parentRelativeTransform'][2][3] += .01
    before = deepcopy(design)
    row, = builder_report(context, design, model, wall_height_m=.074, roof_increment_m=.004)
    assert row['parent_relative_closure_max_abs'] == pytest.approx(.01, abs=1e-7)
    assert design == before and row['task_success'] is None


@pytest.mark.parametrize('case', ['missing_parent', 'ambiguous_key', 'wrong_edge', 'not_upright', 'no_roofs'])
def test_unsupported_input_is_not_reported_as_qualified(assets, case):
    context = native_helpers(SNAPSHOT, assets)
    model = mesh_report(context, assets)
    design = pieces()
    if case == 'missing_parent':
        design[1]['parentId'] = 'absent'
    elif case == 'ambiguous_key':
        design.append(deepcopy(design[0]))
    elif case == 'wrong_edge':
        design[1]['parentEdge'] = 'left'
    elif case == 'not_upright':
        design[1]['n'] = [0., 1., 0.]
    else:
        design = design[:1]
    with pytest.raises(ValueError):
        builder_report(context, design, model, wall_height_m=.074, roof_increment_m=.004)


@pytest.mark.parametrize('field,value', [('wall_height_m', float('nan')),
                                      ('wall_height_m', 0.), ('roof_increment_m', float('inf'))])
def test_nonfinite_geometry_rejected(assets, field, value):
    context = native_helpers(SNAPSHOT, assets)
    kwargs = dict(wall_height_m=.074, roof_increment_m=.004)
    kwargs[field] = value
    with pytest.raises(ValueError):
        builder_report(context, pieces(), mesh_report(context, assets), **kwargs)


def test_nonidentity_scene_graph_rejects_raw_vertex_tip_inference(assets):
    path = primary(assets)
    scene = trimesh.Scene()
    transform = np.eye(4)
    transform[0, 3] = .01
    scene.add_geometry(prism([.074, .0065, .135]), transform=transform)
    path.write_bytes(scene.export(file_type='glb'))
    with pytest.raises(ValueError, match='identity scene-node'):
        mesh_report(native_helpers(SNAPSHOT, assets), assets)


@pytest.mark.parametrize('field,value', [('selected_sha256', 'different'), ('native_sim_scale', 2.),
    ('native_extents_m', [.1, .1, .1]), ('native_tip_needs_local_y_flip', True)])
def test_equal_file_name_does_not_prove_equal_native_asset(assets, field, value):
    primary(assets)
    row = mesh_report(native_helpers(SNAPSHOT, assets), assets)
    assert selection_equivalent(row, deepcopy(row))
    other = dict(row)
    other[field] = value
    assert not selection_equivalent(row, other)


def test_original_coacd_cache_reports_provenance_without_regeneration(tmp_path):
    import hashlib
    mesh = tmp_path / 'source.glb'
    mesh.write_bytes(b'opaque-model-bytes')
    collider = tmp_path / 'source.glb.coacd.ply'
    raw = prism([.074, .0065, .135]).export(file_type='ply')
    comment = b'comment md5=' + hashlib.md5(mesh.read_bytes()).hexdigest().encode() + b', threshold=0.05\n'
    raw = raw.replace(b'element vertex', comment + b'element vertex', 1)
    collider.write_bytes(raw)
    row = collider_report(mesh, collider)
    assert row['cache_source_md5_matches']
    assert row['extents_m'] == pytest.approx([.074, .0065, .135])
    assert not row['regenerated'] and not row['physical_geometry_qualified']
    assert collider.read_bytes() == raw
    mesh.write_bytes(b'changed-model')
    assert not collider_report(mesh, collider)['cache_source_md5_matches']
