import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

from rm75_app.workcell.install_patches import blob_sha
from rm75_app.workcell.migration import export_snapshot, verify_snapshot


def git_bytes(repo: Path, *args: str) -> bytes:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
    ).stdout


def git_text(repo: Path, *args: str) -> str:
    return git_bytes(repo, *args).decode().strip()


def test_audited_overlay_preserves_source_and_is_content_addressed(tmp_path):
    source = tmp_path / "old"
    source.mkdir()
    git_text(source, "init")
    git_text(source, "config", "user.name", "Test")
    git_text(source, "config", "user.email", "test@example.invalid")

    path = source / "pick_jiaobang" / "example.py"
    path.parent.mkdir(parents=True)
    baseline = b'VALUE = "/home/zhangzhao/Desktop/lerobot/pick_jiaobang/base.glb"\n'
    path.write_bytes(baseline)
    git_text(source, "add", ".")
    git_text(source, "commit", "-m", "baseline")
    head = git_text(source, "rev-parse", "HEAD")

    target = tmp_path / "new"
    export_snapshot(
        source,
        target,
        ref=head,
        expected_blobs={"pick_jiaobang/example.py": blob_sha(baseline)},
    )

    dirty = b'VALUE = "/home/zhangzhao/Desktop/lerobot/pick_jiaobang/final.glb"\nFINAL_FIX = True\n'
    path.write_bytes(dirty)
    status_before = git_bytes(source, "status", "--porcelain=v1", "-z")
    overlay = {
        "schema": "rm75.audited_worktree_overlay/v1",
        "source_head": head,
        "source_status_sha256": hashlib.sha256(status_before).hexdigest(),
        "files": [
            {
                "path": "pick_jiaobang/example.py",
                "sha256": hashlib.sha256(dirty).hexdigest(),
                "reason": "synthetic final validated behavior fixture",
            }
        ],
    }
    manifest = tmp_path / "approved.json"
    manifest.write_text(json.dumps(overlay), encoding="utf-8")

    root = Path(__file__).resolve().parents[2]
    env = os.environ.copy()
    env["PYTHONPATH"] = str(root)
    subprocess.run(
        [
            sys.executable,
            str(root / "tools" / "apply_audited_worktree_overlay.py"),
            "--source-repo",
            str(source),
            "--target-repo",
            str(target),
            "--manifest",
            str(manifest),
        ],
        check=True,
        env=env,
        capture_output=True,
        text=True,
    )

    status_after = git_bytes(source, "status", "--porcelain=v1", "-z")
    assert status_after == status_before
    assert path.read_bytes() == dirty

    snapshot = target / "rm75_app" / "_vendor" / "working_snapshot"
    installed = (snapshot / "pick_jiaobang" / "example.py").read_text()
    assert "FINAL_FIX = True" in installed
    assert str(snapshot) in installed

    report = verify_snapshot(snapshot)
    overlay_report = report["source_worktree_overlay"]
    assert overlay_report["source_head"] == head
    assert overlay_report["source_status_sha256"] == hashlib.sha256(status_before).hexdigest()
    assert overlay_report["files"][0]["path"] == "pick_jiaobang/example.py"
    assert report["runtime_gpu_verified"] is False
    assert report["dependency_completeness_verified"] is False
