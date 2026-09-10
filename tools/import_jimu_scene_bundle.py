#!/usr/bin/env python3
"""Read-only import of hash-addressed old Jimu scene bundles into the new repo.

The old repository is only read: its HEAD and worktree status SHA256 are
checked before and after collection, symlinks and path escapes are rejected,
and every imported byte must match the manifest SHA256. Imported copies may
carry explicit recorded relocations (e.g. making a task directory
self-contained); the old files are never modified. Written into
rm75_app/_vendor/jimu_scenes/ with an IMPORT_MANIFEST.json provenance record,
separate from the vendored working snapshot and its MIGRATION_MANIFEST.json.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

SCHEMA='rm75.jimu_scene_import_v1'
MANIFEST_NAME='IMPORT_MANIFEST.json'


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _old_repo_state(source_repo: Path) -> tuple[str, str]:
    head=subprocess.run(['git','-C',str(source_repo),'rev-parse','HEAD'],
        capture_output=True,text=True,check=True).stdout.strip()
    status=subprocess.run(['git','-C',str(source_repo),'status','--porcelain=v1','-z'],
        capture_output=True).stdout
    return head,_sha256(status)


def _load_manifest(path: Path) -> dict:
    manifest=json.loads(path.read_text())
    if manifest.get('schema')!=SCHEMA:
        raise ValueError('Unsupported import manifest schema')
    for key in ('source_repo','source_head','source_status_sha256','destination_root','files'):
        if key not in manifest:
            raise ValueError(f'Import manifest requires {key}')
    if not manifest['files']:
        raise ValueError('Import manifest requires at least one file')
    return manifest


def _resolve_inside(root: Path, relative: str) -> Path:
    if not relative or '..' in Path(relative).parts or Path(relative).is_absolute():
        raise ValueError(f'Path escapes or is absolute: {relative}')
    candidate=root/relative
    real_root=os.path.realpath(root);real=os.path.realpath(candidate)
    if real!=real_root and not real.startswith(real_root+os.sep):
        raise ValueError(f'Path escapes repository root: {relative}')
    # Deliberately NOT .resolve(): resolving follows symlinks and would hide
    # the symlink check in _collect.
    return candidate


def _collect(manifest: dict, source_repo: Path, destination_root: Path, *, verify_only: bool) -> list[dict]:
    records=[]
    for entry in manifest['files']:
        src=_resolve_inside(source_repo,entry['path'])
        if src.is_symlink() or not src.is_file():
            raise ValueError(f'Manifest source is not a regular file: {entry["path"]}')
        data=src.read_bytes()
        if len(data)>manifest.get('max_file_bytes',10*1024*1024):
            raise ValueError(f'Source exceeds size limit: {entry["path"]}')
        actual=_sha256(data)
        if actual!=entry['sha256']:
            raise ValueError(f'SHA256 mismatch for {entry["path"]}: {actual} != {entry["sha256"]}')
        for relocation in manifest.get('relocations',{}).get(entry['path'],[]):
            if relocation.get('kind')!='json_text':
                raise ValueError(f'Unsupported relocation kind for {entry["path"]}')
            before=data;after=before.replace(relocation['from'].encode(),relocation['to'].encode())
            if after==before:
                raise ValueError(f'Relocation text not found in {entry["path"]}: {relocation["from"]}')
            data=after
        relative=entry.get('destination',entry['path'])
        dst=_resolve_inside(destination_root,relative)
        dst.parent.mkdir(parents=True,exist_ok=True)
        if dst.exists() and dst.read_bytes()!=data:
            raise ValueError(f'Destination already exists with different bytes: {dst}')
        if not verify_only:
            dst.write_bytes(data)
        records.append(dict(source_path=entry['path'],destination=relative,
            source_sha256=entry['sha256'],installed_sha256=_sha256(data),
            reason=entry.get('reason',''),relocations=manifest.get('relocations',{}).get(entry['path'],[])))
    return records


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True,type=Path)
    parser.add_argument('--target-repo',default=str(ROOT),type=Path)
    parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    manifest=_load_manifest(args.manifest.resolve())
    target=args.target_repo.resolve()
    if (target/'tools'/'import_jimu_scene_bundle.py').resolve()!=Path(__file__).resolve():
        raise ValueError('--target-repo must be this repository')
    source_repo=Path(manifest['source_repo']).resolve()
    destination_root=_resolve_inside(target,manifest['destination_root'])
    head,status=_old_repo_state(source_repo)
    if head!=manifest['source_head']:
        raise ValueError(f'Old repository HEAD changed: {head} != {manifest["source_head"]}')
    if status!=manifest['source_status_sha256']:
        raise ValueError(f'Old repository worktree status changed: {status} != {manifest["source_status_sha256"]}')
    records=_collect(manifest,source_repo,destination_root,verify_only=args.verify_only)
    head_after,status_after=_old_repo_state(source_repo)
    if (head_after,status_after)!=(head,status):
        raise ValueError('Old repository state changed during collection; nothing was safe to record')
    record=dict(schema=SCHEMA,source_repo=str(source_repo),source_head=head,source_status_sha256=status,
        destination_root=manifest['destination_root'],files=records,
        old_repository_modified=False,write_attempted=not args.verify_only)
    if not args.verify_only:
        (destination_root/MANIFEST_NAME).write_text(json.dumps(record,indent=2,ensure_ascii=False)+'\n')
        print(f'Imported {len(records)} files into {destination_root}',file=sys.stderr)
    else:
        existing=json.loads((destination_root/MANIFEST_NAME).read_text())
        for field in ('source_head','source_status_sha256','destination_root'):
            if existing.get(field)!=record[field]:
                raise ValueError(f'Installed provenance mismatch on {field}')
        for row,installed in zip(records,existing['files']):
            if {k:row[k] for k in ('source_path','destination','source_sha256','installed_sha256')}!=\
               {k:installed[k] for k in ('source_path','destination','source_sha256','installed_sha256')}:
                raise ValueError(f'Installed provenance mismatch for {row["source_path"]}')
        print(f'Verified {len(records)} files in {destination_root}',file=sys.stderr)
    print(json.dumps(dict(files=len(records),verify_only=args.verify_only,
        source_head=head,status_sha256=status)))


if __name__=='__main__':
    main()
