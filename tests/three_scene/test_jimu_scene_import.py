"""Read-only old Jimu scene bundle import: hashes, relocations, provenance."""
import hashlib
import json
import subprocess
from pathlib import Path

import pytest

from rm75_app.magnetic.design import validate_design

REPO=Path(__file__).resolve().parents[2]
MANIFEST_PATH=REPO/'configs/workcell/jimu_scenes_import_20260910.json'
DEST=REPO/'rm75_app/_vendor/jimu_scenes'


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def test_import_manifest_is_complete_and_self_consistent():
    manifest=json.loads(MANIFEST_PATH.read_text())
    assert manifest['schema']=='rm75.jimu_scene_import_v1'
    assert manifest['destination_root']=='rm75_app/_vendor/jimu_scenes'
    paths=[entry['path'] for entry in manifest['files']]
    assert 'Beta_demo-codex-v0.9/jimu_tasks/tag1_standard_three_layer/manifest.json' in paths
    assert 'Beta_demo-codex-v0.9/jimu_tasks/tag2_arc_base/manifest.json' in paths
    for entry in manifest['files']:
        assert 'sha256' in entry and 'reason' in entry
    assert manifest['source_head']=='36798efbd12814841607951c9af470b309b34fd3'


def test_installed_files_match_manifest_hashes_without_external_checkout():
    manifest=json.loads(MANIFEST_PATH.read_text())
    installed=json.loads((DEST/'IMPORT_MANIFEST.json').read_text())
    assert installed['old_repository_modified'] is False
    for row in installed['files']:
        data=(DEST/row['destination']).read_bytes()
        if row['relocations']:
            # Relocated copies differ from the old-repo source by design.
            assert _sha256(data)==row['installed_sha256']!=row['source_sha256'], row['source_path']
        else:
            assert _sha256(data)==row['source_sha256']==row['installed_sha256'], row['source_path']
    assert sum(1 for row in installed['files'] if row['relocations'])==1
    from tools.import_jimu_scene_bundle import verify_installed
    verify_installed(manifest,DEST)


def test_imported_scenes_are_valid_designs_with_relocated_tag2_manifest():
    tasks=DEST/'Beta_demo-codex-v0.9/jimu_tasks'
    tag1=json.loads((tasks/'tag1_standard_three_layer/builder_scene.json').read_text())
    tag2=json.loads((tasks/'tag2_arc_base/builder_scene.json').read_text())
    assert validate_design(tag1).ordered_roles is not None
    assert validate_design(tag2).ordered_roles is not None
    assert len(tag1['pieces'])==21 and len(tag2['pieces'])==19
    tag2_manifest=json.loads((tasks/'tag2_arc_base/manifest.json').read_text())
    ref=tag2_manifest['sam6d_fixed_scene_result_file']
    assert ref=='full_scene_pose_results_tip_up.json'
    assert (tasks/'tag2_arc_base'/ref).is_file()


def test_collect_rejects_symlink_escape_and_hash_mismatch(tmp_path):
    from tools.import_jimu_scene_bundle import _collect,_resolve_inside
    source=tmp_path/'src';source.mkdir();(source/'a.json').write_text('{"x":1}')
    target=tmp_path/'dst';target.mkdir()
    manifest=dict(files=[dict(path='a.json',sha256=_sha256(b'{"x":1}'),reason='r')],
        relocations={},max_file_bytes=10485760)
    records=_collect(manifest,source,target,verify_only=False)
    assert records[0]['installed_sha256']==_sha256(b'{"x":1}')
    with pytest.raises(ValueError,match='SHA256 mismatch'):
        _collect(dict(files=[dict(path='a.json',sha256=_sha256(b'other'),reason='r')],
            relocations={},max_file_bytes=10485760),source,target,verify_only=False)
    with pytest.raises(ValueError,match='escapes'):
        _resolve_inside(source,'../outside.json')
    with pytest.raises(ValueError,match='absolute'):
        _resolve_inside(source,'/etc/passwd')
    (source/'link.json').symlink_to(source/'a.json')
    with pytest.raises(ValueError,match='not a regular file'):
        _collect(dict(files=[dict(path='link.json',sha256=_sha256(b'{"x":1}'),reason='r')],
            relocations={},max_file_bytes=10485760),source,target,verify_only=False)


def test_collect_applies_relocation_only_to_listed_copy(tmp_path):
    from tools.import_jimu_scene_bundle import _collect
    source=tmp_path/'src';source.mkdir()
    (source/'m.json').write_text('{"f":"../../out/x.json"}')
    target=tmp_path/'dst';target.mkdir()
    manifest=dict(files=[dict(path='m.json',sha256=_sha256(b'{"f":"../../out/x.json"}'),reason='r')],
        relocations={'m.json':[dict(kind='json_text',from_='../../out/x.json'.replace('_',''),
            to='x.json',reason='self-contained')]},max_file_bytes=10485760)
    # The 'from_' key hack above is deliberate: 'from' is a Python keyword in
    # dict() calls, so the tool manifest uses 'from'; rebuild the dict literally.
    manifest['relocations']['m.json']=[dict(kind='json_text',**{'from':'../../out/x.json','to':'x.json','reason':'self-contained'})]
    records=_collect(manifest,source,target,verify_only=False)
    assert records[0]['installed_sha256']==_sha256(b'{"f":"x.json"}')
    assert (source/'m.json').read_text()=='{"f":"../../out/x.json"}'


def test_verify_only_detects_installed_provenance_mismatch(tmp_path):
    from tools.import_jimu_scene_bundle import _collect
    source=tmp_path/'src';source.mkdir();(source/'a.json').write_text('{"x":1}')
    target=tmp_path/'dst';target.mkdir()
    manifest=dict(files=[dict(path='a.json',sha256=_sha256(b'{"x":1}'),reason='r')],
        relocations={},max_file_bytes=10485760)
    _collect(manifest,source,target,verify_only=False)
    (target/'a.json').write_text('{"x":2}')
    with pytest.raises(ValueError,match='different bytes'):
        _collect(manifest,source,target,verify_only=False)
