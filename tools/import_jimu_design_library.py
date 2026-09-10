#!/usr/bin/env python3
"""Read two original task bundles, export a new bounded Jimu generation library.

Never execute legacy scripts, rewrite a source, infer an arc radius or copy the
whole dirty worktree. Multiple --grid-task/--arc-task arguments add original
shape templates. The output must be outside every source task directory.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
from pathlib import Path

from rm75_app.magnetic.design import validate_design
from rm75_app.magnetic.generation import validate_library
from rm75_app.workcell.io import read_json, atomic_json, digest

BUILDER_FLAGS = {
    'outward_clearance_m': '--jimu-builder-outward-clearance-m',
    'outward_clearance_max_depth': '--jimu-builder-outward-clearance-max-depth',
    'layer_z_extra_m': '--jimu-builder-layer-z-extra-m',
    'roof_uniform_preplace_height_m': '--jimu-roof-uniform-preplace-height-m',
    'final_contact_low_hover_height_m': '--jimu-final-contact-low-hover-height-m',
}
BOOL_FLAGS = {
    'canonicalize_outward_normals': '--jimu-builder-canonicalize-outward-normals',
    'use_design_parent_targets': '--jimu-builder-use-design-parent-targets',
}
TAG_FLAGS = {key: '--jimu-apriltag-' + key.replace('_', '-') for key in (
    'base_id', 'base_size_m', 'base_yaw_deg', 'tray_id', 'tray_size_m', 'tray_yaw_deg',
    'base_world_offset_x_m', 'base_world_offset_y_m', 'tray_world_offset_x_m', 'tray_world_offset_y_m')}


def import_task(directory, board_id, index):
    directory = Path(directory).expanduser().resolve()
    manifest_file = directory / 'manifest.json'
    manifest = read_json(manifest_file, max_bytes=500_000)
    if manifest.get('schema')!='jimu_task_manifest_v1': raise ValueError('Unknown original task manifest schema')
    raw = manifest.get('builder_scene_json')
    if not isinstance(raw, str) or not raw: raise ValueError('Original manifest lacks builder_scene_json')
    design_path = (directory / raw).resolve()
    if not design_path.is_relative_to(directory): raise ValueError('Builder scene leaves original task bundle')
    design = read_json(design_path, max_bytes=1_500_000); validate_design(design)
    locked = [p for p in design['pieces'] if p.get('locked', False)]
    moving = [p for p in design['pieces'] if not p.get('locked', False)]
    options = []
    builder = manifest.get('builder', {})
    for key, flag in BUILDER_FLAGS.items():
        if key in builder:
            value = builder[key]
            if isinstance(value, bool) or not isinstance(value, (float, int)):
                raise ValueError('Malformed original numeric builder option')
            options += [flag, str(value)]
    for key, flag in BOOL_FLAGS.items():
        if key in builder:
            if type(builder[key]) is not bool: raise ValueError('Malformed original boolean option')
            options.append(flag if builder[key] else '--no-' + flag[2:])
    for key, flag in TAG_FLAGS.items():
        if key in manifest.get('apriltag', {}):
            value = manifest['apriltag'][key]
            if isinstance(value, bool) or not isinstance(value, (float, int)):
                raise ValueError('Malformed original tag configuration')
            options += [flag, str(value)]
    tray = manifest.get('tray', {}).get('slot_layout') or builder.get('tray_slot_role_order')
    if tray:
        roles = [x.get('role') if isinstance(x, dict) else x for x in tray]
        if any(not isinstance(x, str) or not x for x in roles): raise ValueError('Invalid original tray roles')
        options += ['--jimu-tray-slot-role-order', *roles]
    recipe = dict(native_args=options,task_manifest=manifest)
    fixed = manifest.get('sam6d_fixed_scene_result_file')
    hashes = {'manifest': hashlib.sha256(manifest_file.read_bytes()).hexdigest(),
              'builder': hashlib.sha256(design_path.read_bytes()).hexdigest()}
    if not fixed: raise ValueError('Original task requires its own fixed scene for SIM')
    if fixed:
        fixed_path = (directory / fixed).resolve()
        if not fixed_path.is_relative_to(directory): raise ValueError('Fixed scene leaves original task bundle')
        fixed_payload=read_json(fixed_path)
        if not isinstance(fixed_payload.get('results'),list) or not fixed_payload['results']:
            raise ValueError('Original fixed task scene has no camera pose results')
        recipe.update(fixed_scene=str(fixed_path), fixed_scene_sha256=hashlib.sha256(fixed_path.read_bytes()).hexdigest())
    return dict(id=f'{board_id}_{index:02d}', title=str(manifest.get('name') or directory.name)[:100],
        design=design, design_digest=digest(design), locked_digest=digest(locked),
        inventory=dict(Counter(p['type'] for p in moving)),
        support_dependencies={}, native_recipe=recipe,
        provenance=dict(source_task=str(directory), source_sha256=hashes,
                        geometry='exact_original_not_reconstructed', source_modified=False))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grid-task', type=Path, action='append', required=True)
    parser.add_argument('--arc-task', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.output.expanduser().resolve()
    if output.exists(): raise FileExistsError('Choose a new library output; existing files are never overwritten')
    boards = {}
    for key, sources, title in (('grid_3x3', args.grid_task, '原版 3×3 九块底板'), ('arc', args.arc_task, '原版弧形底板')):
        if len(sources) > 24: raise ValueError('Too many source task bundles')
        for source in sources:
            if output.is_relative_to(source.expanduser().resolve()):
                raise ValueError('Output must not modify an original task directory')
        boards[key] = dict(title=title, templates=[import_task(p, key, i) for i, p in enumerate(sources)])
    library = validate_library(dict(schema='rm75_jimu_design_library_v1', boards=boards))
    atomic_json(output, library)
    print(f'Imported original bases to {output}; no native/GPU/hardware execution.')

if __name__ == '__main__': main()
