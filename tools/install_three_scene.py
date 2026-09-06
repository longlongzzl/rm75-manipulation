#!/usr/bin/env python3
"""Install additive three-task code, then optionally migrate the working sources.

No force-overwrite, no Git checkout changes. Existing new-package files must be
byte-identical or installation stops before any files are copied.
"""
from __future__ import annotations
import argparse,hashlib,shutil,sys
from pathlib import Path
PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE))
from rm75_app.workcell.io import atomic_json,dumps
from rm75_app.workcell.migration import export_snapshot
from rm75_app.workcell.install_patches import prepare,apply


def install(target,source=None):
    target=Path(target).resolve()
    if not (target/'rm75_app/cli.py').is_file():
        raise ValueError('Target is not an existing rm75-manipulation checkout')
    if target==PACKAGE:
        raise ValueError('Install into the actual repository, not the overlay itself')
    patches=prepare(target)
    roots=('rm75_app','tools','tests','docs','examples')
    files=[p for prefix in roots for p in (PACKAGE/prefix).rglob('*')
           if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc']
    conflicts=[str(p.relative_to(PACKAGE)) for p in files
               if (target/p.relative_to(PACKAGE)).exists() and (target/p.relative_to(PACKAGE)).read_bytes()!=p.read_bytes()]
    if conflicts:
        raise FileExistsError(f'Refusing to replace locally modified files: {conflicts}')
    for path in files:
        dest=target/path.relative_to(PACKAGE);dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,dest)
    manifest={'overlay_files':[str(p.relative_to(PACKAGE)) for p in files],
              'original_planners_overwritten':False,'source_migration':None}
    if source is not None:
        manifest['source_migration']=export_snapshot(source,target)
    apply(target,patches)
    manifest['integration_files']=list(patches)
    atomic_json(target/'runtime_data/workcell/install_manifest.json',manifest)
    print(dumps({'installed_files':len(files),'source_copied':source is not None,'target':str(target)}))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target',type=Path,required=True)
    p.add_argument('--legacy-source',type=Path)
    a=p.parse_args();install(a.target,a.legacy_source)

if __name__=='__main__':
    main()
