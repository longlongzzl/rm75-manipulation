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
from pathlib import Path, PurePosixPath

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rm75_app.workcell.io import atomic_json, read_json
from rm75_app.workcell.migration import EXCLUDED, EXTENSIONS, relocate, selected, verify_snapshot

MANIFEST_NAME = "MIGRATION_MANIFEST.json"
OVERLAY_SCHEMA = "rm75.audited_worktree_overlay/v1"
AUDIT_ONLY_ROOTS = ("jimu_builder_frontend/", "jimu_tasks/", "Demo_Triangle/")
AUDIT_ONLY_FILES = {"FoundationPose/run_demo_scale.py"}


def _run(repo: Path, *args: str) -> bytes:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
    ).stdout


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _overlay_selected(path: str) -> bool:
    if selected(path):
        return True
    candidate = PurePosixPath(path)
    if candidate.is_absolute() or ".." in candidate.parts or EXCLUDED.intersection(candidate.parts):
        return False
    if any("失败" in part or "备份" in part for part in candidate.parts):
        return False
    permitted = path in AUDIT_ONLY_FILES or path.startswith(AUDIT_ONLY_ROOTS)
    return bool(permitted and candidate.suffix.lower() in EXTENSIONS)


def _load_overlay(path: Path) -> dict:
    payload = read_json(path)
    if payload.get("schema") != OVERLAY_SCHEMA:
        raise ValueError("unsupported overlay manifest schema")
    files = payload.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("overlay manifest requires a non-empty files list")
    seen: set[str] = set()
    for item in files:
        path_value = str(item.get("path") or "")
        if path_value in seen:
            raise ValueError(f"duplicate overlay path: {path_value}")
        seen.add(path_value)
    return payload


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repo", type=Path, required=True)
    parser.add_argument("--target-repo", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args(argv)

    source = args.source_repo.expanduser().resolve()
    target = args.target_repo.expanduser().resolve()
    manifest_path = args.manifest.expanduser().resolve()
    overlay = _load_overlay(manifest_path)

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
    base = read_json(root / MANIFEST_NAME)
    if base.get("source_worktree_overlay"):
        raise RuntimeError(
            "snapshot already has a worktree overlay; recreate the fixed baseline "
            "instead of stacking overlays"
        )
    if not isinstance(base.get("files"), list):
        raise ValueError("fixed snapshot manifest has an unexpected files format")

    base_by_path = {
        str(item["path"]): dict(item)
        for item in base["files"]
        if isinstance(item, dict) and item.get("path")
    }
    approved: dict[str, bytes] = {}
    source_sizes: dict[str, int] = {}
    relocated_counts: dict[str, int] = {}
    reasons: dict[str, str] = {}

    for item in overlay["files"]:
        path = str(item.get("path") or "")
        if not _overlay_selected(path):
            raise ValueError(f"path is not permitted by migration policy: {path}")
        expected = str(item.get("sha256") or "").lower()
        if len(expected) != 64 or any(ch not in "0123456789abcdef" for ch in expected):
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
        raw = source_path.read_bytes()
        if len(raw) > 80_000_000:
            raise ValueError(f"overlay file too large: {path}")
        actual = _sha256(raw)
        if actual != expected:
            raise RuntimeError(f"overlay file changed since audit: {path}")

        output, changes = relocate(raw, path, root)
        approved[path] = output
        source_sizes[path] = len(raw)
        relocated_counts[path] = int(changes)
        reasons[path] = reason

    # Recheck before writing anything to the new repository. The old repository
    # must remain byte-for-byte the same status snapshot used by the audit.
    if _sha256(_run(source, "status", "--porcelain=v1", "-z")) != status_sha:
        raise RuntimeError("source worktree changed during overlay collection")

    for path, data in approved.items():
        destination = root / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)

    updated_by_path = dict(base_by_path)
    for path, data in approved.items():
        previous = base_by_path.get(path, {})
        updated_by_path[path] = {
            "path": path,
            "source_blob": None,
            "base_source_blob": previous.get("source_blob"),
            "source_bytes": source_sizes[path],
            "installed_sha256": _sha256(data),
            "relocated_literals": relocated_counts[path],
            "worktree_overlay": True,
            "audit_reason": reasons[path],
        }

    # Preserve original baseline order; append genuinely new audited dependencies.
    original_order = [str(item["path"]) for item in base["files"]]
    extra = sorted(set(updated_by_path) - set(original_order))
    base["files"] = [updated_by_path[path] for path in [*original_order, *extra]]
    base["file_count"] = len(base["files"])
    base["source_worktree_overlay"] = {
        "schema": OVERLAY_SCHEMA,
        "source_head": head,
        "source_status_sha256": status_sha,
        "audit_manifest": str(manifest_path),
        "files": [
            {
                "path": path,
                "sha256": updated_by_path[path]["installed_sha256"],
                "reason": reasons[path],
            }
            for path in sorted(approved)
        ],
    }
    # Local GPU/dependency validation must be rerun after any overlay.
    base["runtime_gpu_verified"] = False
    base["dependency_completeness_verified"] = False
    atomic_json(root / MANIFEST_NAME, base)

    report = verify_snapshot(root)
    print(
        json.dumps(
            {
                "ok": True,
                "overlay_files": len(approved),
                "source_head": head,
                "source_status_sha256": status_sha,
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
