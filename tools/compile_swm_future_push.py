#!/usr/bin/env python3
"""Compile an original saved push candidate into explicitly planned native FK."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.swm.future_tool import compile_future_tool_motion
from rm75_app.workcell.io import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('plan','transition','geometry','urdf','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    from tools.run_network_isolated import block_network
    block_network()
    values={}
    for name in ('plan','transition','geometry'):
        path=getattr(args,name)
        if path.stat().st_size>16000000:raise ValueError('Future input exceeds budget')
        values[name]=json.loads(path.read_bytes())
    args.output.mkdir(parents=True,exist_ok=False)
    result=compile_future_tool_motion(values['plan'],values['transition']['initial_snapshot'],values['geometry'],args.urdf)
    atomic_json(args.output/'future_tool_motion.json',result)
    summary={key:value for key,value in result.items() if key!='samples'}
    summary.update(samples=len(result['samples']),duration_s=result['samples'][-1]['time_s'],
                   tool_links=len(values['geometry']['links']),tool_shapes=sum(map(len,values['geometry']['links'].values())))
    atomic_json(args.output/'summary.json',summary)
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
