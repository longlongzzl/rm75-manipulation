"""Release validation does not depend on an external user's checkout."""
import json
import subprocess
import pytest
from tools import import_jimu_scene_bundle as importer


@pytest.fixture
def source_bundle(tmp_path):
    source=tmp_path/'source';source.mkdir()
    def git(*args):
        return subprocess.run(['git','-C',str(source),*args],check=True,capture_output=True)
    git('init','-q')
    (source/'scene.json').write_text('{"pose":1}')
    git('add','scene.json')
    git('-c','user.name=Fixture','-c','user.email=fixture@example.invalid','-c','core.hooksPath=/dev/null',
        'commit','-qm','fixture')
    # Keep the source already dirty: a second byte change must still be detected.
    (source/'scene.json').write_text('{"pose":2}')
    head,status=importer._old_repo_state(source)
    manifest=dict(schema=importer.SCHEMA,source_repo=str(source),source_head=head,source_status_sha256=status,
        destination_root='bundle',files=[dict(path='scene.json',sha256=importer._sha256(b'{"pose":2}'),reason='fixture')],
        relocations={})
    destination=tmp_path/'bundle'
    return source,destination,manifest


def test_release_verification_survives_later_source_edit_or_removal(source_bundle):
    source,destination,manifest=source_bundle
    record=importer.import_checked(manifest,source,destination)
    (destination/importer.MANIFEST_NAME).write_text(json.dumps(record))
    (source/'scene.json').write_text('{"pose":3}')
    assert importer.verify_installed(manifest,destination)==record
    (source/'scene.json').unlink()
    assert importer.verify_installed(manifest,destination)==record
    (destination/'scene.json').write_text('{"pose":999}')
    with pytest.raises(ValueError,match='corrupted'):
        importer.verify_installed(manifest,destination)


def test_import_detects_source_byte_change_even_when_git_status_is_identical(source_bundle,monkeypatch):
    source,destination,manifest=source_bundle
    collect=importer._collect
    def changed(*args,**kwargs):
        rows=collect(*args,**kwargs)
        (source/'scene.json').write_text('{"pose":3}')
        return rows
    monkeypatch.setattr(importer,'_collect',changed)
    with pytest.raises(ValueError,match='Source changed'):
        importer.import_checked(manifest,source,destination)
    assert not (destination/importer.MANIFEST_NAME).exists()


def test_import_preserves_source_content_and_git_state(source_bundle):
    source,destination,manifest=source_bundle
    before=importer._old_repo_state(source)
    data=(source/'scene.json').read_bytes()
    importer.import_checked(manifest,source,destination)
    assert importer._old_repo_state(source)==before
    assert (source/'scene.json').read_bytes()==data


def test_source_git_errors_are_not_hashed_as_empty_state(tmp_path):
    with pytest.raises(subprocess.CalledProcessError):
        importer._old_repo_state(tmp_path)
