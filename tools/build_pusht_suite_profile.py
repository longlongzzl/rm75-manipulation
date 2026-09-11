#!/usr/bin/env python3
"""Build the machine profile for tools/run_pusht_random_suite.py --run.

Reuses the same frozen base input and unchanged thresholds as
tools/run_pusht_physics_validation.py; the suite itself samples the cases.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--response-calibration',type=Path,default=None)
    parser.add_argument('--geometry-id',default='original')
    parser.add_argument('--seed',type=int,default=77)
    args=parser.parse_args()
    output=args.output.resolve()
    if output.exists():raise FileExistsError('Choose a new profile output path')
    source=ROOT/'runtime_data/three_scene/pickplace_pusht_followup_20260908/gpu_baseline/input.json'
    frozen=json.loads(source.read_text())
    profile=json.loads((ROOT/'examples/workcell/machine.example.json').read_text())
    model={**frozen['config'],'maximum_push_length_m':.05,'horizon':3,'beam_width':12}
    if args.response_calibration is not None:
        calibration=json.loads(args.response_calibration.read_text())
        model['response_fits']=calibration['response_fits']
    motion=dict(frozen['motion']);motion['retreat_clearance_m']=.05
    profile['pusht']['model']=model
    profile['pusht']['physics']=dict(enabled_backends=['tool_only_physics','full_arm_physics'],
        simulation_python='/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python',
        planner_python='/home/zhangzhao/anaconda3/envs/curobo2/bin/python',motion=motion,
        initial_pose=[.35,-.18,0.],gravity_compensation='original-agent',
        gripper_feedback_geometry=True,
        randomization_bounds=list(frozen['config'].get('workspace',[.15,.65,-.30,.30])),
        geometry_variants={'wide':dict(bar_width_m=.12)},
        planner=dict(gripper_collision_closed_joint_position=.9,
                     ignore_gripper_internal_self_collision=True))
    assert profile['hardware']['hardware_reviewed'] is False
    profile['pusht']['physics'].setdefault('geometry_variants',{})[args.geometry_id]=dict()
    profile['suite']=dict(seed=args.seed,count=12,geometry_id=args.geometry_id,
                          original_t_geometry=True,execute_real=False,hardware_connected=False)
    atomic_json(output,profile)
    print(f'Profile written to {output}; frozen base sha256 of model config: '
          f'{__import__("hashlib").sha256(json.dumps(frozen["config"],sort_keys=True).encode()).hexdigest()[:16]}')


if __name__=='__main__':main()
