import hashlib
import json
from pathlib import Path

import pytest

from tools.apply_audited_worktree_overlay import approved_bytes


@pytest.fixture
def roots(tmp_path):
    source, archive = tmp_path / 'old', tmp_path / 'archive'
    source.mkdir(); archive.mkdir()
    return source, archive


def test_archive_restores_only_exact_previously_approved_bytes(roots):
    source, archive = roots
    (source / 'entry.py').write_bytes(b'new unapproved hardware change')
    (archive / 'entry.py').write_bytes(b'approved')
    assert approved_bytes(source, archive, 'entry.py', hashlib.sha256(b'approved').hexdigest()) == (
        b'approved', 'approved_archive')
    assert (source / 'entry.py').read_bytes() == b'new unapproved hardware change'


def test_matching_old_source_remains_preferred(roots):
    source, archive = roots
    (source / 'entry.py').write_bytes(b'approved')
    (archive / 'entry.py').write_bytes(b'not approved')
    assert approved_bytes(source, archive, 'entry.py', hashlib.sha256(b'approved').hexdigest())[1] == 'old_worktree'


@pytest.mark.parametrize('archive_content', [None, b'changed', b'relocated approved'])
def test_unapproved_or_relocated_archive_is_rejected(roots, archive_content):
    source, archive = roots
    (source / 'entry.py').write_bytes(b'changed')
    if archive_content is not None:
        (archive / 'entry.py').write_bytes(archive_content)
    with pytest.raises(RuntimeError, match='approved raw SHA unavailable'):
        approved_bytes(source, archive, 'entry.py', hashlib.sha256(b'approved').hexdigest())


def test_no_implicit_archive_fallback(roots):
    source, archive = roots
    (archive / 'entry.py').write_bytes(b'approved')
    with pytest.raises(RuntimeError):
        approved_bytes(source, None, 'entry.py', hashlib.sha256(b'approved').hexdigest())


@pytest.mark.parametrize('location', ['source', 'archive'])
def test_symlinks_are_not_approved_bytes(roots, location):
    source, archive = roots
    outside = source.parent / 'outside'
    outside.write_bytes(b'approved')
    ((source if location == 'source' else archive) / 'entry.py').symlink_to(outside)
    with pytest.raises(ValueError, match='symlink'):
        approved_bytes(source, archive, 'entry.py', hashlib.sha256(b'approved').hexdigest())


def test_parent_escape_rejected(roots):
    source, archive = roots
    with pytest.raises(ValueError):
        approved_bytes(source, archive, '../outside', hashlib.sha256(b'approved').hexdigest())


def test_restored_triangle_dependencies_are_exact_approved_assets():
    root = Path(__file__).resolve().parents[2]
    snapshot = root / 'rm75_app/_vendor/working_snapshot'
    overlay = json.loads((root / 'configs/workcell/approved_worktree_overlay_20260907.json').read_text())
    manifest = json.loads((snapshot / 'MIGRATION_MANIFEST.json').read_text())
    expected = {
        'Demo_Triangle/red_triangle_74x135x6p5.glb':
            'fac68dee16a4f8d5b17d7de0f4007ac2445472c70d338c46dbb98908e38f4175',
        'Demo_Triangle/red_triangle_74x135x6p5.glb.coacd.ply':
            'f6d655ec58ef7db661241935c6d7f9ee3d0ac35c6807e0d2d1b57f9772582a12',
    }
    approved = {r['path']: r['sha256'] for r in overlay['files']}
    installed = {r['path']: r for r in manifest['files']}
    for name, digest in expected.items():
        assert approved[name] == digest
        assert installed[name]['installed_sha256'] == digest
        assert installed[name]['relocated_literals'] == 0
        assert hashlib.sha256((snapshot / name).read_bytes()).hexdigest() == digest
    # The unrelated unapproved old hardware edits must not enter this rebuild.
    arm = 'lerobot/common/robots/realman_lerobot/realman_arm.py'
    assert hashlib.sha256((snapshot / arm).read_bytes()).hexdigest() == approved[arm]
