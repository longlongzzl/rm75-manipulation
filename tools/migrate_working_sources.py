#!/usr/bin/env python3
"""Install or verify the committed PickPlace/Jimu working snapshot.

This command never reads dirty worktree bytes. Use
``tools/apply_audited_worktree_overlay.py`` afterwards for explicitly reviewed
current-worktree fixes.
"""
from pathlib import Path
import argparse
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rm75_app.workcell.io import dumps
from rm75_app.workcell.migration import SOURCE_COMMIT, export_snapshot, verify_snapshot


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repo", type=Path)
    parser.add_argument("--target-repo", type=Path, required=True)
    parser.add_argument("--source-ref", default=SOURCE_COMMIT)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args(argv)

    snapshot_root = (
        args.target_repo.expanduser().resolve()
        / "rm75_app"
        / "_vendor"
        / "working_snapshot"
    )
    if args.verify_only:
        report = verify_snapshot(snapshot_root)
    else:
        if args.source_repo is None:
            parser.error("--source-repo is required unless --verify-only is used")
        report = export_snapshot(
            args.source_repo,
            args.target_repo,
            ref=args.source_ref,
        )

    print(
        dumps(
            {
                "source_commit": report.get("source_commit"),
                "files": report["file_count"],
                "frontend_candidates": report.get("frontend_candidates", []),
                "runtime_gpu_verified": bool(
                    report.get("runtime_gpu_verified", False)
                ),
                "dependency_completeness_verified": bool(
                    report.get("dependency_completeness_verified", False)
                ),
                "source_worktree_overlay": report.get("source_worktree_overlay"),
            }
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
