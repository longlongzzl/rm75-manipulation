#!/usr/bin/env python3
"""Read-only saved return-start sphere/mesh intersections; no shared world writes."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.transforms import quaternion_matrix


def audit(path):
    raw=path.read_bytes()
    row=next(json.loads(line)['evidence'] for line in raw.splitlines()
             if json.loads(line).get('evidence',{}).get('event')=='jimu_return_query_diagnostic')
    spheres=row['endpoints']['start']['curobo_raw_world_collision']['nonzero'];output=[]
    for obj in row['world_objects']:
        if not obj.get('file_path'):continue
        source=Path(obj['file_path']).resolve();source.relative_to(ROOT/'rm75_app/_vendor/working_snapshot')
        mesh=trimesh.load(source,force='scene').to_geometry();mesh.apply_scale(obj['scale'])
        transform=np.eye(4);transform[:3,:3]=quaternion_matrix(obj['pose'][3:]);transform[:3,3]=obj['pose'][:3]
        mesh.apply_transform(transform)
        for sphere in spheres:
            _,distance,_=trimesh.proximity.closest_point_naive(mesh,[sphere['center']])
            clearance=float(distance[0]-sphere['radius'])
            # A negative unsigned surface gap PROVES intersection, regardless
            # of inside/outside. A positive gap alone does not certify clearance.
            output.append(dict(obstacle=obj['name'],link=sphere['link'],sphere=sphere['sphere'],
                surface_distance_minus_radius_m=clearance,surface_intersection_proven=clearance<0,
                mesh_sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    return dict(source_events_sha256=hashlib.sha256(raw).hexdigest(),source_step=row['step_id'],
        released=row['released'],source=row['source'],mode='proposed_post_release_return_start',
        no_new_ik=True,world_not_modified=True,rows=output)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--events',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    result=audit(args.events);atomic_json(args.output,result)
    print(json.dumps(result))
