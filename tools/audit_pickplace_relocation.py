#!/usr/bin/env python3
"""Read-only CPU PickPlace asset/input equivalence and saved margin evidence.

No native task, solver, robot, camera, model change, or collision ablation.
"""
import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.workcell.pickplace_curobo_only import source_adapter
from rm75_app.workcell.transport_contact import payload_contact_config
from tools.run_native_pickplace import FIXED_CASES


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_specs(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def compare_scene(old, new):
    if not isinstance(old.get('objects'), dict) or not old['objects']:
        raise ValueError('Original scene needs nonempty objects')
    if not isinstance(new.get('objects'), dict) or not new['objects']:
        raise ValueError('Current scene needs nonempty objects')
    old_meta, new_meta = old.get('generated_metadata', {}), new.get('generated_metadata', {})
    top = [k for k in sorted(old.keys() | new.keys()) if old.get(k) != new.get(k)]
    meta = [k for k in sorted(old_meta.keys() | new_meta.keys()) if old_meta.get(k) != new_meta.get(k)]
    names = sorted(old['objects'].keys() | new['objects'].keys())
    return dict(json_equal=old == new, object_count_old=len(old['objects']),
        object_count_new=len(new['objects']), objects_equal=old['objects'] == new['objects'],
        changed_top_level_keys=top, changed_generated_metadata_keys=meta,
        changed_object_names=[k for k in names if old['objects'].get(k) != new['objects'].get(k)],
        provenance_path_only_difference=top == ['generated_metadata'] and meta == ['base_scene_file'])


def margin_summary(row, buffers):
    """Algebra on recorded native overlap, not a new less-conservative query."""
    if (row.get('event') != 'pickplace_failed_lift_ik_diagnostic'
            or row.get('diagnostic_complete') is not True or row.get('state_unchanged') is not True
            or row.get('native_success') is not False or row.get('attached') is not True):
        raise ValueError('Expected a complete, unchanged, failed native attached-lift diagnostic')
    values = [float(buffers[name]) for name in ('base_link', 'attached_object')]
    if not all(math.isfinite(v) and v >= 0 for v in values):
        raise ValueError('Invalid original self buffers')
    nominal = row['nominal_goal_payload_base']
    if (nominal.get('geometric_necessary_condition_only') is not True
            or nominal.get('physical_geometry_qualified') is not False):
        raise ValueError('Unexpected nominal geometry qualification')
    contacts = []
    for contact in nominal['contacts']:
        if ({contact['link_a'], contact['link_b']} != {'base_link', 'attached_object'}
                or contact.get('pair_ignored') is not False):
            raise ValueError('Not an enforced payload/base pair')
        overlap = float(contact['overlap_m'])
        if not math.isfinite(overlap) or overlap <= 0:
            raise ValueError('Invalid recorded positive overlap')
        gap = sum(values) - overlap
        contacts.append(dict(sphere_i=contact['sphere_i'], sphere_j=contact['sphere_j'],
            enforced_overlap_m=overlap, link_self_buffer_sum_m=sum(values),
            gap_without_link_self_buffers_m=gap,
            underlying_spheres_still_overlap=gap < 0,
            original_query_remains_failed=True))
    if not contacts:
        raise ValueError('No recorded nominal contacts; do not pass an empty audit')
    start = np.asarray(row['start']['fk']['position'], dtype=float)
    goal = np.asarray(row['goal_pose']['position'], dtype=float)
    if start.shape != (3,) or goal.shape != (3,) or not np.isfinite([start, goal]).all():
        raise ValueError('Invalid recorded FK/goal position')
    delta = goal - start
    return dict(source=row['source'], step_id=row['step_id'],
        collision_model_sha256=row['collision_model_sha256'],
        original_native_success=False, state_unchanged=True, physical_geometry_qualified=False,
        safe_to_execute=False, new_collision_query=False,
        recorded_goal_minus_start_fk_z_m=float(delta[2]),
        recorded_goal_minus_start_fk_xy_norm_m=float(np.linalg.norm(delta[:2])),
        contacts=contacts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-repo', type=Path, required=True)
    parser.add_argument('--lift-evidence', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Do not overwrite previous evidence')
    old = args.source_repo.resolve()
    snapshot = ROOT / 'rm75_app/_vendor/working_snapshot'
    manifest = verify_snapshot(snapshot)
    status = lambda: subprocess.check_output(['git', '-C', str(old), 'status', '--porcelain=v1', '-z'],
        env={**os.environ, 'GIT_OPTIONAL_LOCKS': '0'})
    before = status()
    specs_rel = Path('pick_jiaobang/object_specs.py')
    if digest(old / specs_rel) != digest(snapshot / specs_rel):
        raise ValueError('Unreviewed original object spec change')
    old_specs = load_specs(old / specs_rel, '_pickplace_old_asset_specs')
    with source_adapter(snapshot):
        new_specs = load_specs(snapshot / specs_rel, '_pickplace_new_asset_specs')
    scenes, names = [], set()
    for case, (relative, requested) in FIXED_CASES.items():
        new_path = ROOT / relative
        old_path = old / 'pick_jiaobang/test_scenes' / relative.split('test_scenes/', 1)[1]
        a, b = json.loads(old_path.read_text()), json.loads(new_path.read_text())
        names.update(a['objects']); names.update(b['objects'])
        scenes.append(dict(case=case, requested_objects=list(requested),
            old_sha256=digest(old_path), current_sha256=digest(new_path),
            bytes_equal=old_path.read_bytes() == new_path.read_bytes(), **compare_scene(a, b)))
    assets = []
    for name in sorted(names):
        a, b = old_specs.get_object_spec(name), new_specs.get_object_spec(name)
        if a is None or b is None:
            raise ValueError('Missing object spec: ' + name)
        left, right = asdict(a), asdict(b)
        pairs = []
        for kind in ('mesh_file', 'sim_asset_file'):
            pa = Path(left.pop(kind) or a.mesh_file).expanduser().resolve()
            pb = Path(right.pop(kind) or b.mesh_file).expanduser().resolve()
            if not pa.is_relative_to(old) or not pb.is_relative_to(snapshot):
                raise ValueError('Asset escaped its repository')
            pairs.append(dict(kind=kind, old_relative_path=str(pa.relative_to(old)),
                current_relative_path=str(pb.relative_to(snapshot)),
                old_sha256=digest(pa), current_sha256=digest(pb), bytes_equal=pa.read_bytes() == pb.read_bytes()))
        sa, sb = old_specs.resolve_object_spec_scales(a), new_specs.resolve_object_spec_scales(b)
        assets.append(dict(object=name, nonpath_spec_fields_equal=left == right,
            scales_equal=sa == sb, original_scales=list(sa), current_scales=list(sb), paths=pairs))
    cfg_rel = Path('pick_jiaobang/curobo_rm75_config/rm75.yml')
    cfg = yaml.safe_load((snapshot / cfg_rel).read_text())
    old_cfg = yaml.safe_load((old / cfg_rel).read_text())
    kin = cfg.get('robot_cfg', cfg)['kinematics']
    old_kin = old_cfg.get('robot_cfg', old_cfg)['kinematics']
    checked_cfg = payload_contact_config(cfg)
    checked = checked_cfg.get('robot_cfg', checked_cfg)
    if kin['self_collision_buffer'] != old_kin['self_collision_buffer'] or checked['kinematics']['self_collision_buffer'] != kin['self_collision_buffer']:
        raise ValueError('Cannot infer recorded margin from changed original buffers')
    buffers = kin['self_collision_buffer']
    fixed_cfg = yaml.safe_load(subprocess.check_output(['git', '-C', str(old), 'show',
        manifest['source_commit'] + ':' + str(cfg_rel)], text=True))
    fixed_buffers = fixed_cfg.get('robot_cfg', fixed_cfg)['kinematics']['self_collision_buffer']
    if fixed_buffers != buffers:
        raise ValueError('Current self buffers differ from the fixed source baseline')
    evidence = []
    for path in args.lift_evidence:
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        if not rows:
            raise ValueError('Empty saved lift evidence')
        evidence.append(dict(input_sha256=digest(path), rows=[margin_summary(row, buffers) for row in rows]))
    after = status()
    if before != after:
        raise RuntimeError('Old worktree status changed during audit')
    report = dict(schema='rm75.pickplace_relocation_audit/v1',
        tested_base_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        audit_source_sha256=digest(Path(__file__)), execute_real=False, hardware_connected=False,
        camera_captured=False, new_gpu_run=False, migration_performed=False, task_success=None,
        old_status_sha256_before=hashlib.sha256(before).hexdigest(), old_status_sha256_after=hashlib.sha256(after).hexdigest(),
        snapshot_verified_files=len(manifest['files']), object_specs_sha256=digest(snapshot / specs_rel),
        original_buffers_preserved=True, base_self_buffer_m=float(buffers['base_link']),
        buffer_provenance_fixed_commit=manifest['source_commit'],
        fixed_source_buffers_equal=True,
        payload_self_buffer_m=float(buffers['attached_object']),
        collision_config_sha256=digest(snapshot / cfg_rel),
        collision_policy_source_sha256=digest(ROOT / 'rm75_app/workcell/transport_contact.py'),
        scenes=scenes, assets=assets, saved_lift_margin_analysis=evidence,
        notes=['No model, target, candidate, seed, sphere, buffer, or ignore pair was changed.',
               'Margin decomposition is derived from saved GPU overlap and unchanged configuration, not a new validity query.',
               'Gap without LINK self buffers still includes any sphere inflation already in the native sphere model.',
               'Positive sphere gap is not clearance from physical meshes or permission to move; original failures remain failures.',
               'Only object data equivalence is asserted for scenes with relocated base_scene_file provenance.',
               'This audit does not establish the complete dynamic dependency closure, task success, or hardware qualification.'])
    atomic_json(args.output, report)
    print(json.dumps(dict(scenes=len(scenes), objects=len(assets),
        all_objects_equal=all(row['objects_equal'] for row in scenes),
        all_assets_equal=all(row['nonpath_spec_fields_equal'] and row['scales_equal'] and all(p['bytes_equal'] for p in row['paths']) for row in assets),
        saved_lift_diagnostics=sum(len(item['rows']) for item in evidence), new_gpu_run=False)))


if __name__ == '__main__':
    main()
