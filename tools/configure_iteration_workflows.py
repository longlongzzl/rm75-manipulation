#!/usr/bin/env python3
"""Create a NEW per-machine profile; never change the existing qualified one."""
from __future__ import annotations
import argparse
import copy
from pathlib import Path
from rm75_app.workcell.io import read_json,atomic_json
from rm75_app.magnetic.generation import load_library


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True,type=Path)
    parser.add_argument('--library',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--enable-paired-repair',action='store_true')
    parser.add_argument('--add-sim-variants',action='store_true')
    args=parser.parse_args(argv)
    if args.output.exists() or args.output.resolve()==args.profile.resolve():
        raise FileExistsError('Choose a new output; original profile is retained')
    profile=copy.deepcopy(read_json(args.profile))
    if profile.get('schema')!='rm75_workcell_machine_v1':raise ValueError('Not a workcell profile')
    profile.setdefault('magnetic',{})['design_library']=str(args.library.resolve())
    library=load_library(profile)
    if set(library['boards'])!={'grid_3x3','arc'}:
        raise ValueError('Import BOTH original base families before configuring the workflow')
    if args.enable_paired_repair:
        section=profile.setdefault('pickplace',{})
        if (section.get('fixed_scene_format')!='native_world' or
                section.get('simulation_contact_policy')!='transport_world_checked_compatibility'):
            raise ValueError('Repair requires a checked native-world SIM profile')
        section['paired_endpoint_repair']={'enabled':True,'max_queries':12,'budget_s':5.}
    if args.add_sim_variants:
        variants=profile.setdefault('pusht',{}).setdefault('physics',{}).setdefault('geometry_variants',{})
        for key,value in {
            'wide_t':{'bar_width_m':.12,'bar_height_m':.03,'stem_width_m':.03,'stem_height_m':.07},
            'long_stem_t':{'bar_width_m':.10,'bar_height_m':.03,'stem_width_m':.03,'stem_height_m':.09},
            'small_t':{'bar_width_m':.085,'bar_height_m':.025,'stem_width_m':.025,'stem_height_m':.06}
        }.items():
            variants.setdefault(key,value)
    # New simulation workflows cannot inherit positive hardware claims accidentally.
    profile.setdefault('hardware',{})['hardware_reviewed']=False
    for key in ('pickplace','magnetic','pusht'):
        profile.setdefault(key,{})['integration_qualified']=False
    atomic_json(args.output,profile)
    print('Created simulation-only workflow profile:',args.output)
    return 0

if __name__=='__main__':raise SystemExit(main())
