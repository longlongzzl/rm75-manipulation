#!/usr/bin/env python3
"""Overlay explicitly audited dirty-worktree files on a fixed source snapshot.

The old repository is treated as read-only. Only files listed with exact SHA256
hashes and audit reasons in the manifest are copied. The same migration filters
and path relocation rules are reused, Python sources are parsed before writing,
and the final snapshot manifest is re-verified.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rm75_app.workcell.io import atomic_json, read_json
from rm75_app.workcell.migration import MANIFEST, relocate, selected, verify_snapshot


def _run(repo: Path, *args: str) -> bytes:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
    ).stdout


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _load_manifest(path: Path) -> dict:
    payload = read_json(path)
    if payload.get("schema") != "rm75.audited_worktree_overlay/v1":
        raise ValueError("unsupported overlay manifest schema")
    files = payload.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("overlay manifest requires a non-empty files list")
    return payload


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repo", type=Path, required=True)
    parser.add_argument("--target-repo", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args(argv)

    source = args.source_repo.expanduser().resolve()
    target = args.target_repo.expanduser().resolve()
    overlay = _load_manifest(args.manifest.expanduser().resolve())

    head = _run(source, "rev-parse", "HEAD").decode().strip()
    if head != overlay.get("source_head"):
        raise RuntimeError(
            f"source HEAD changed: {head} != {overlay.get('source_head')}"
        )

    status_before = _run(source, "status", "--porcelain=v1", "-z")
    status_sha = _sha256(status_before)
    expected_status_sha = str(overlay.get("source_status_sha256") or "")
    if expected_status_sha and status_sha != expected_status_sha:
        raise RuntimeError("source worktree status changed since audit")

    root = target / "rm75_app" / "_vendor" / "working_snapshot"
    base = read_json(root / MANIFEST)
    if base.get("source_worktree_overlay"):
        raise RuntimeError(
            "snapshot already has a worktree overlay; recreate the fixed baseline "
            "instead of stacking overlays"
        )

    approved: dict[str, bytes] = {}
    reasons: dict[str, str] = {}
    for item in overlay["files"]:
        path = str(item.get("path") or "")
        if not selected(path):
            raise ValueError(f"path is not permitted by migration policy: {path}")
        expected = str(item.get("sha256") or "").lower()
        if len(expected) != 64:
            raise ValueError(f"invalid sha256 for {path}")
        reason = str(item.get("reason") or "").strip()
        if not reason:
            raise ValueError(f"missing audit reason for {path}")

        source_path = (source / path).resolve()
        try:
            source_path.relative_to(source)
        except ValueError as exc:
            raise ValueError(f"path escapes source repo: {path}") from exc
        if not source_path.is_file() or source_path.is_symlink():
            raise ValueError(f"overlay path is not a regular file: {path}")
        data = source_path.read_bytes()
        if len(data) > 80 * 1024 * 1024:
            raise ValueError(f"overlay file too large: {path}")
        actual = _sha256(data)
        if actual != expected:
            raise RuntimeError(f"overlay file changed since audit: {path}")

        approved[path] = relocate(path, data, root)
        reasons[path] = reason

    if _sha256(_run(source, "status", "--porcelain=v1", "-z")) != status_sha:
        raise RuntimeError("source worktree changed during overlay collection")

    for path, data in approved.items():
        destination = root / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)

    files = dict(base.get("files", {}))
    for path, data in approved.items():
        files[path] = {
            "source_blob": None,
            "installed_sha256": _sha256(data),
            "size": len(data),
            "worktree_overlay": True,
            "audit_reason": reasons[path],
        }
    base["files"] = files
    base["source_worktree_overlay"] = {
        "schema": overlay["schema"],
        "source_head": head,
        "source_status_sha256": status_sha,
        "audit_manifest": str(args.manifest.expanduser().resolve()),
        "files": [
            {
                "path": path,
                "sha256": files[path]["installed_sha256"],
                "reason": reasons[path],
            }
            for path in approved
        ],
    }
    # Local GPU/dependency validation must be rerun after any overlay.
    base["runtime_gpu_verified"] = False
    base["dependency_completeness_verified"] = False
    atomic_json(root / MANIFEST, base)

    report = verify_snapshot(root)
    print(
        json.dumps(
            {
                "ok": True,
                "overlay_files": len(approved),
                "source_head": head,
                "snapshot_files": report["file_count"],
                "runtime_gpu_verified": False,
                "dependency_completeness_verified": False,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
