#!/usr/bin/env python3
"""Copy committed working PickPlace/Jimu implementation into the new repository."""
from pathlib import Path
import argparse,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rm75_app.workcell.migration import export_snapshot
from rm75_app.workcell.io import dumps


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-repo',type=Path,required=True)
    p.add_argument('--target-repo',type=Path,required=True)
    a=p.parse_args()
    report=export_snapshot(a.source_repo,a.target_repo)
    print(dumps({'source_commit':report['source_commit'],'files':report['file_count'],
                 'frontend_candidates':report['frontend_candidates'],'runtime_gpu_verified':False}))

if __name__=='__main__':
    main()
