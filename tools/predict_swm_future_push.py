#!/usr/bin/env python3
"""Run original native planned-tool dynamics in private CPU PhysX worlds."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.swm.physics_replay import SubprocessReplayWorld
from rm75_app.swm.scene import digest
from rm75_app.workcell.io import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('transition','future-motion','geometry','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--python',required=True);args=parser.parse_args()
    from tools.run_network_isolated import block_network
    block_network()
    def read(path):
        if path.stat().st_size>16000000:raise ValueError('Prediction input exceeds budget')
        return json.loads(path.read_bytes())
    transition=read(args.transition);motion=read(args.future_motion);geometry=read(args.geometry)
    snapshot=transition['initial_snapshot']
    action=dict(source='planned_trajectory',time_s=[r['time_s'] for r in motion['samples']],
        T_world_tcp=[r['T_world_tcp'] for r in motion['samples']],stages=[r['stage'] for r in motion['samples']])
    args.output.mkdir(parents=True,exist_ok=False);results=[]
    for name,sf,df,density in [('low',.15,.1,1000.),('nominal',.3,.3,1000.),('high',.6,.6,1000.),('dense',.3,.3,2000.)]:
        request=dict(mode='future_prediction',hypothesis_id=name,
            parameters=dict(static_friction=sf,dynamic_friction=df,density_kg_m3=density),
            initial_snapshot=snapshot,object_id=transition['object_id'],planned_action=action,
            plan_action_digest=digest(action),future_tool_motion=motion,sample_times=[0.,action['time_s'][-1]])
        world=SubprocessReplayWorld(request,python=args.python,native_tool_geometry=geometry,directory=args.output)
        try:result=world.replay()
        finally:world.close()
        if result.get('mode')!='future_prediction' or result.get('identification_eligible') is not False:
            raise ValueError('Future result mislabeled as measured identification')
        results.append({k:v for k,v in result.items() if k!='predicted_tool_readback'})
    report=dict(scope='original_saved_candidate_native_future_dynamics',results=results,
        source_snapshot_id=snapshot['snapshot_id'],source_plan_digest=motion['source_plan_digest'],
        measured_action=False,posterior_updated=False,online_worker_consumption=False,
        execution_authorized=False,hardware_connected=False)
    atomic_json(args.output/'summary.json',report)
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
