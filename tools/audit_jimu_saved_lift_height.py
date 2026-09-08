#!/usr/bin/env python3
"""Read saved invalid spheres against the original triangle mesh; no robot/IK.

Rigidly raising one reported sphere is a geometric sensitivity query, not a
reachable arm pose, a full robot audit, or permission to change the lift height.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.transforms import quaternion_matrix


def audit(events_path):
    raw=events_path.read_bytes()
    row=next(json.loads(line)['evidence'] for line in raw.splitlines()
             if json.loads(line).get('evidence',{}).get('event')=='jimu_read_only_collision_diagnostic')
    obj=next(obj for obj in row['world_objects'] if obj['name']=='scene_obstacle_right_roof_triangle')
    path=Path(obj['file_path']).resolve()
    path.relative_to(ROOT/'rm75_app/_vendor/working_snapshot')
    if path.name!='red_triangle_74x135x6p5.glb':raise ValueError('Expected the approved original triangle')
    mesh_bytes=path.read_bytes()
    if hashlib.sha256(mesh_bytes).hexdigest()!='fac68dee16a4f8d5b17d7de0f4007ac2445472c70d338c46dbb98908e38f4175':
        raise ValueError('Triangle differs from approved SHA')
    mesh=trimesh.load(path,force='scene').to_geometry()
    mesh.apply_scale(obj['scale'])
    transform=np.eye(4);transform[:3,:3]=quaternion_matrix(obj['pose'][3:]);transform[:3,3]=obj['pose'][:3]
    mesh.apply_transform(transform)
    rows=[]
    for sphere in row['curobo_raw_world_collision']['nonzero']:
        center=np.array(sphere['center']);radius=sphere['radius']
        if center[2]<=mesh.bounds[1,2]:raise ValueError('This outside-distance check needs center above mesh bound')
        for dz in (0.,.005,.01,.02):
            point=center+[0,0,dz]
            nearest,distance,face=trimesh.proximity.closest_point_naive(mesh,[point])
            rows.append(dict(link=sphere['link'],hypothetical_world_z_translation_m=dz,
                             sphere_bottom_minus_mesh_top_m=float(point[2]-radius-mesh.bounds[1,2]),
                             sphere_mesh_clearance_m=float(distance[0]-radius)))
    return dict(diagnostic_only=True,execute_real=False,new_gpu_planning=False,full_robot_qualified=False,
                source_step_id=row['step_id'],source_events_sha256=hashlib.sha256(raw).hexdigest(),
                mesh_sha256=hashlib.sha256(mesh_bytes).hexdigest(),mesh_height_m=float(mesh.extents[2]),
                mesh_top_z_m=float(mesh.bounds[1,2]),rows=rows,
                limitation='Saved native sphere versus original GLB surface; translation sensitivity is NOT an IK/path/full-world test')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--events',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    result=audit(args.events);atomic_json(args.output,result);print(json.dumps(result))


if __name__=='__main__':main()
