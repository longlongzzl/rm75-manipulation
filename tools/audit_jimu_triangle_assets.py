#!/usr/bin/env python3
"""Read-only CPU audit of the native triangle entry's optional model branch.

Does not install assets, run a native task, import the robot entry, or open SDKs.
Only selected geometry helpers are compiled from the verified snapshot source.
"""
import argparse
import ast
from dataclasses import replace
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot
from tools.run_workcell_native_validation import read_original_task_bundle

ENTRY = Path('Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py')
PRIMARY = Path('Demo_Triangle/red_triangle_74x135x6p5.glb')
FALLBACK = Path('pick_jiaobang/meshs/red_triangle.glb')
HELPERS = (
    '_triangle_geometry_mesh_and_scale', '_triangle_extents',
    '_triangle_tip_needs_local_y_flip', '_triangle_tip_up_local_rotation',
    '_demo_triangle_spec', '_builder_piece_center', '_builder_piece_matrix',
    '_builder_matrix_from_json', '_builder_parent_relative_matrix',
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def native_helpers(snapshot, asset_root):
    """Use native selection/scale/frame code, without executing entry imports."""
    spec_path = snapshot / 'pick_jiaobang/object_specs.py'
    name = '_rm75_triangle_asset_audit_specs'
    spec = importlib.util.spec_from_file_location(name, spec_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    base = replace(module.get_object_spec('red_triangle_front'),
                   mesh_file=str(asset_root / FALLBACK),
                   sim_asset_file=str(asset_root / FALLBACK))
    tree = ast.parse((snapshot / ENTRY).read_text())
    functions = {node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)}
    if any(name not in functions for name in HELPERS):
        raise ValueError('Native geometry helper closure changed')
    constants = [node for node in tree.body if isinstance(node, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'DEFAULT_TRIANGLE_EXTENTS_M'
                         for t in node.targets)]
    if len(constants) != 1:
        raise ValueError('Native triangle extent fallback missing or ambiguous')
    selected = ast.Module(body=[ast.ImportFrom(module='__future__',
        names=[ast.alias(name='annotations')], level=0), *constants,
        *(functions[name] for name in HELPERS)], type_ignores=[])
    context = dict(np=np, trimesh=trimesh, Path=Path, replace=replace,
        DEMO_TRIANGLE_MESH=asset_root / PRIMARY,
        portable=SimpleNamespace(object_specs=SimpleNamespace(
            get_object_spec=lambda name: base if name == 'red_triangle_front' else None,
            resolve_object_spec_scales=module.resolve_object_spec_scales)))
    exec(compile(ast.fix_missing_locations(selected), str(snapshot / ENTRY), 'exec'), context)
    return context


def mesh_report(context, asset_root):
    path, scale = context['_triangle_geometry_mesh_and_scale']()
    path = Path(path).resolve()
    if not path.is_relative_to(asset_root.resolve()) or not path.is_file():
        raise ValueError('Selected native mesh is not a file inside its asset root')
    loaded = trimesh.load(path, force='scene')
    extents = np.asarray(context['_triangle_extents'](), dtype=float)
    if extents.shape != (3,) or not np.isfinite(extents).all() or np.min(extents) <= 0:
        raise ValueError('Invalid measured triangle dimensions')
    needs_flip = bool(context['_triangle_tip_needs_local_y_flip']())
    tip = np.asarray([0., 0., -1. if needs_flip else 1.])
    node_identity = all(np.allclose(loaded.graph[node][0], np.eye(4), atol=1e-7)
                        for node in loaded.graph.nodes_geometry)
    if not node_identity:
        raise ValueError('Raw-vertex tip audit requires identity scene-node transforms')
    applied_spec = context['_demo_triangle_spec']('red_triangle_front')
    return dict(selected_relative_path=str(path.relative_to(asset_root.resolve())),
        selected_sha256=digest(path), selected_bytes=path.stat().st_size,
        native_sim_scale=float(scale), native_extents_m=extents.tolist(),
        native_tip_needs_local_y_flip=needs_flip,
        mesh_tip_local_z_sign=int(tip[2]), scene_node_transforms_identity=node_identity,
        native_standard_tip_dot_parent_up=float(
            (context['_triangle_tip_up_local_rotation']() @ tip)[2]),
        native_spec_uses_primary=Path(applied_spec.sim_asset_file).resolve() ==
                                 (asset_root / PRIMARY).resolve(),
        physical_geometry_qualified=False)


def builder_report(context, pieces, mesh, *, wall_height_m, roof_increment_m):
    """Compare input frame closure and nominal edge gaps; do not alter targets."""
    if not np.isfinite([wall_height_m, roof_increment_m]).all() or wall_height_m <= 0:
        raise ValueError('Invalid original wall height or layer increment')
    lookup = {}
    for piece in pieces:
        for key in ('id', 'role'):
            if piece.get(key):
                if piece[key] in lookup and lookup[piece[key]] is not piece:
                    raise ValueError('Ambiguous builder piece key')
                lookup[piece[key]] = piece
    rows = []
    for piece in pieces:
        if piece.get('type') != 'triangle' or piece.get('locked', False):
            continue
        parent = lookup.get(piece.get('parentId'))
        if parent is None:
            raise ValueError('Triangle parent is missing')
        child_matrix = context['_builder_piece_matrix'](piece)
        parent_matrix = context['_builder_piece_matrix'](parent)
        relative = context['_builder_parent_relative_matrix'](piece, parent)
        closure = parent_matrix @ relative
        tip = child_matrix[:3, :3] @ np.asarray([0., 0., mesh['mesh_tip_local_z_sign']])
        # This input is top/bottom plate stacking; reject other semantics.
        if (piece.get('parentEdge') != 'top' or piece.get('childAttachEdge') != 'bottom'
                or not np.allclose(parent_matrix[:3, 2], [0., 1., 0.], atol=1e-6)
                or not np.allclose(child_matrix[:3, 2], [0., 1., 0.], atol=1e-6)):
            raise ValueError('Not an upright top/bottom builder triangle')
        relative_height = float(closure[1, 3] - parent_matrix[1, 3])
        edge_gap = relative_height - .5 * (wall_height_m + mesh['native_extents_m'][2])
        rows.append(dict(role=piece.get('role'), parent_role=parent.get('role'),
            parent_relative_closure_max_abs=float(np.max(np.abs(closure - child_matrix))),
            mesh_tip_dot_builder_up=float(tip[1]),
            design_parent_child_height_m=relative_height,
            nominal_bbox_edge_gap_before_layer_extra_m=edge_gap,
            nominal_bbox_edge_gap_with_original_layer_extra_m=edge_gap + roof_increment_m,
            original_layer_extra_m=roof_increment_m,
            necessary_geometry_only=True, task_success=None))
    if not rows:
        raise ValueError('No triangle task pieces observed')
    return rows


def collider_report(mesh, collider):
    """Read the original cache, never regenerate or shrink its convex geometry."""
    comments = []
    with collider.open('rb') as stream:
        for _ in range(100):
            line = stream.readline(4096)
            if not line or line.strip() == b'end_header':
                break
            if line.startswith(b'comment '):
                comments.append(line.decode('ascii', errors='replace'))
        else:
            raise ValueError('Unsupported collision PLY header')
    matches = re.findall(r'\bmd5=([a-f0-9]{32})\b', ''.join(comments))
    loaded = trimesh.load(collider, force='scene')
    # MD5 here is the existing SAPIEN cache provenance, not a security digest.
    source_md5 = hashlib.md5(mesh.read_bytes()).hexdigest()
    return dict(source_mesh_md5=source_md5,
        cache_source_md5_matches=len(matches) == 1 and matches[0] == source_md5,
        full_native_cache_reuse_verified=False,
        extents_m=np.asarray(loaded.bounds[1] - loaded.bounds[0]).tolist(),
        regenerated=False, physical_geometry_qualified=False)


def selection_equivalent(left, right):
    return all(left[key] == right[key] for key in (
        'selected_sha256', 'native_sim_scale', 'native_extents_m',
        'native_tip_needs_local_y_flip'))


def source_status(source):
    return subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain=v1', '-z'],
        env={**os.environ, 'GIT_OPTIONAL_LOCKS': '0'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-repo', type=Path, required=True)
    parser.add_argument('--task-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.source_repo.resolve()
    snapshot = ROOT / 'rm75_app/_vendor/working_snapshot'
    if args.output.exists():
        raise FileExistsError('Refusing to overwrite an existing audit')
    verification = verify_snapshot(snapshot)
    before = source_status(source)
    entry_matches = digest(source / ENTRY) == digest(snapshot / ENTRY)
    if not entry_matches:
        raise ValueError('Old entry differs; native helper equivalence needs a separate source audit')
    files, task_hashes = read_original_task_bundle(args.task_dir)
    builder = json.loads(files['builder'].read_text())
    manifest = json.loads(files['manifest'].read_text())
    increments = manifest['builder']['layer_z_extra_m']
    if len(increments) != 3:
        raise ValueError('Expected the original three-layer manifest')
    wall_asset = Path('Beta_demo-codex-v0.9/jimu_portable_repro/assets/red_jimu_plate_74x6p5x74.glb')
    wall = trimesh.load(snapshot / wall_asset, force='scene')
    wall_height = float((wall.bounds[1] - wall.bounds[0])[2])
    reports = {}
    for label, asset_root in [('old_worktree', source), ('snapshot', snapshot)]:
        context = native_helpers(snapshot, asset_root)
        model = mesh_report(context, asset_root)
        reports[label] = dict(mesh=model, builder_roofs=builder_report(context, builder['pieces'],
            model, wall_height_m=wall_height, roof_increment_m=float(increments[2])))
    candidates = []
    approved = json.loads((ROOT / 'configs/workcell/approved_worktree_overlay_20260907.json').read_text())
    approved_hashes = {row['path']: row['sha256'] for row in approved['files']}
    for relative in (PRIMARY, Path(str(PRIMARY) + '.coacd.ply')):
        path = source / relative
        tracked = subprocess.check_output(['git', '-C', str(source), 'ls-files', '--stage', '--', str(relative)])
        candidates.append(dict(path=str(relative), sha256=digest(path), bytes=path.stat().st_size,
            old_git_tracked=bool(tracked), present_in_snapshot=(snapshot / relative).exists(),
            classification='candidate_final_fix',
            listed_in_existing_approved_overlay=approved_hashes.get(str(relative)) == digest(path)))
    after = source_status(source)
    unchanged = before == after and all(digest(files[k]) == v for k, v in task_hashes.items())
    report = dict(schema='rm75.jimu_triangle_asset_audit/v1',
        tested_base_commit=subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
        audit_source_sha256=digest(Path(__file__)),
        execute_real=False, hardware_connected=False, camera_captured=False,
        cpu_geometry_only=True, gpu_rerun=False, task_success=None,
        migration_performed=False, old_worktree_and_task_unchanged=unchanged,
        old_status_sha256_before=hashlib.sha256(before).hexdigest(),
        old_status_sha256_after=hashlib.sha256(after).hexdigest(),
        native_entry_sha256=digest(snapshot / ENTRY),
        old_entry_matches_verified_snapshot=entry_matches,
        snapshot_verified_file_count=len(verification['files']),
        snapshot_manifest_sha256=digest(snapshot / 'MIGRATION_MANIFEST.json'),
        original_task_hashes=task_hashes, original_wall_height_m=wall_height,
        original_wall_asset_sha256=digest(snapshot / wall_asset),
        compared_asset_selection_equivalent=selection_equivalent(
            reports['old_worktree']['mesh'], reports['snapshot']['mesh']),
        original_collision_cache=collider_report(source / PRIMARY, source / (str(PRIMARY) + '.coacd.ply')),
        candidates=candidates, environments=reports,
        notes=['No mesh, collider, builder pose, solver, or native runtime was changed.',
               'Standard tip-up correction is distinct from direct builder-axis placement.',
               'Bounding-box edge gaps are not magnetic capture or physical contact qualification.',
               'Cache source MD5 matching alone does not certify all SAPIEN decomposition parameters.',
               'Optional asset omission changes a native branch despite a manifest-consistent snapshot.',
               'Asset inclusion needs explicit approval and subsequent full GPU regression.'])
    if not unchanged:
        raise RuntimeError('Old repository or task changed during read-only audit')
    atomic_json(args.output, report)
    print(json.dumps(dict(output=args.output.name, asset_equivalent=report['compared_asset_selection_equivalent'],
        old_unchanged=unchanged, migration_performed=False)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
