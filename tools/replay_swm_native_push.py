#!/usr/bin/env python3
"""Replay one immutable native-worker transition; no new reference action.

Original closed-gripper planning spheres remain an explicitly approximate tool
model. This experiment does not authorize execution or update the live SWM.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.swm.identification import IsolatedReplayPool,infer_posterior,validate_transition
from rm75_app.swm.physics_replay import SubprocessReplayWorld
from rm75_app.workcell.io import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transition',type=Path,required=True)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--native-motion',type=Path)
    parser.add_argument('--original-urdf',type=Path)
    parser.add_argument('--python',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from tools.run_network_isolated import block_network
    block_network()
    if args.transition.stat().st_size>16000000 or args.geometry.stat().st_size>1000000:
        raise ValueError('Native replay input exceeds evidence budget')
    raw=args.transition.read_bytes();transition=validate_transition(json.loads(raw))
    if (transition['domain']!='physics' or transition['observation_source']!='native_primary_PhysX_readback'
            or not transition.get('physical_clock_evidence')):
        raise ValueError('An actual native-worker physical-clock transition is required')
    geometry_raw=args.geometry.read_bytes();geometry=json.loads(geometry_raw)
    motion=None
    if args.native_motion:
        if not args.original_urdf:raise ValueError('Original native URDF construction source required')
        if args.native_motion.stat().st_size>16000000:raise ValueError('Native motion exceeds evidence budget')
        motion=json.loads(args.native_motion.read_bytes())
    elif geometry.get('ready') is not True or geometry.get('execute_real') is not False:
        raise ValueError('Original simulation planner geometry required')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    hypotheses=[dict(id=name,parameters=dict(static_friction=sf,dynamic_friction=df,density_kg_m3=density))
        for name,sf,df,density in [('low_friction',.15,.1,1000.),('nominal',.3,.3,1000.),
                                  ('high_friction',.6,.6,1000.),('other_density',.3,.3,2000.)]]
    def factory(request):
        if motion is not None:
            request=dict(request,native_tool_construction=dict(urdf=str(args.original_urdf.resolve()),
                urdf_sha256=hashlib.sha256(args.original_urdf.read_bytes()).hexdigest()))
            return SubprocessReplayWorld(request,python=args.python,directory=output,timeout_s=180,
                                         native_tool_geometry=geometry,native_tool_motion=motion)
        return SubprocessReplayWorld(request,python=args.python,tool_spheres=geometry['spheres'],
                                     directory=output,timeout_s=180)
    checked,rollouts=IsolatedReplayPool(factory,workers=1).run(transition,hypotheses)
    posterior=infer_posterior(checked,hypotheses,rollouts)
    report=dict(scope='native_worker_measured_action_replay_with_approximate_original_tool_spheres',
        transition_sha256=hashlib.sha256(raw).hexdigest(),geometry_sha256=hashlib.sha256(geometry_raw).hexdigest(),
        transition_digest=checked['transition_digest'],action_digest=checked['action_digest'],
        actual_action_id=checked['action_id'],tool_model='original_closed_gripper_planning_spheres_approximation',
        native_tool_geometry_qualified=False,full_arm_replayed=False,live_swm_updated=False,
        next_planner_consumed_posterior=False,hardware_connected=False,model_inference_run=False,
        hypotheses=hypotheses,posterior=posterior,
        rollouts=[{key:row.get(key) for key in ('hypothesis_id','valid','parameters','action_digest',
            'transition_digest','initial_snapshot_id','engine','native_target_mass_kg',
            'native_target_collision_shapes','time_s','T_world_object')} for row in rollouts])
    if motion is not None:
        report.update(scope='native_worker_measured_link_action_replay',
            tool_model='original_native_convex_shapes_measured_per_link',
            native_tool_geometry_qualified=all(row.get('valid') is True for row in rollouts),
            native_tool_motion_digest=motion['motion_digest'],
            native_geometry_readback=[{key:row.get(key) for key in
                ('hypothesis_id','native_tool_geometry_digest','native_tool_motion_digest','native_tool_shape_count')}
                for row in rollouts])
    atomic_json(output/'summary.json',report)
    print(json.dumps(report,allow_nan=False),flush=True)
    return 0 if all(row.get('valid') is True for row in rollouts) else 1


if __name__=='__main__':raise SystemExit(main())
