#!/usr/bin/env python3
"""Audit a fixed-world -> native camera fixture conversion; never capture or move.

An output SAM6D-schema fixture is emitted ONLY if the unchanged native mapping
(including its original table clamp) reproduces every fixed world pose. This is
format/mapping evidence, not a new SAM6D inference or physical calibration test.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json,read_json
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.workcell.transforms import rigid


def inverse_native_affine_pose(world_pose, calibration, robot_base, offset, local_rotation,
                               *, direct_calibration=False, map_robot_base=True):
    """Inverse of the reviewed affine portion, without bypassing table checks."""
    world=rigid(world_pose,'world pose').copy()
    calibration=rigid(calibration,'calibration')
    robot_base=rigid(robot_base,'robot base')
    local=rigid(local_rotation,'local rotation')
    offset=np.asarray(offset,dtype=float)
    if offset.shape!=(3,) or not np.isfinite(offset).all():raise ValueError('Invalid original position offset')
    world[:3,3]-=offset
    camera_to_base=calibration if direct_calibration else np.linalg.inv(calibration)
    base=robot_base if map_robot_base else np.eye(4)
    return np.linalg.inv(camera_to_base)@np.linalg.inv(base)@world@np.linalg.inv(local)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--extensions',type=Path,required=True)
    args=parser.parse_args();scene=args.scene.resolve();output=args.output.resolve()
    extensions=args.extensions.resolve()
    output.mkdir(parents=True,exist_ok=False)
    original=read_json(scene)
    if not isinstance(original.get('objects'),dict) or 'gluestick' not in original['objects']:
        raise ValueError('Use an original complete gluestick fixed scene')
    root=ROOT/'rm75_app/_vendor/working_snapshot';verify_snapshot(root)
    calibration_path=ROOT/'assets/calibration/camera_extrinsic_opencv.npy'
    report={'execute_real':False,'camera_captured':False,'planner_executed':False,
        'source_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),
        'calibration_sha256':hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
        'all_objects_equivalent':False,'objects':[]}
    from rm75_app.workcell.pickplace_curobo_only import source_adapter
    from rm75_app.workcell.legacy import import_working_entry
    env=None
    try:
        with source_adapter(root):
            os.chdir(root)
            sys.argv=['native-camera-fixture','--object-name','gluestick','--render-mode','none',
                '--curobo-torch-extensions-dir',str(extensions),
                '--camera-extrinsic-opencv-path',str(calibration_path)]
            module=import_working_entry(root,'pickplace')
            parsed=module.parse_args();base=module.direct.targeted.base
            native,_=base.make_cycle_args(parsed,'gluestick')
            assert native.execute_real is False and native.skip_foundationpose is False
            bridge=base.load_module_from_path('native_camera_fixture_bridge',native.bridge_script_path)
            native.env_id=bridge.ensure_pick_jiaobang_env_registered(native.env_id,native.extra_maniskill_package_root)
            base.patch_maniskill_compute_angle_between_device_mismatch()
            env=bridge.gym.make(native.env_id,robot_uids='RM75',obs_mode='none',control_mode='pd_joint_pos',
                render_mode='none',max_episode_steps=native.max_episode_steps,
                object_asset_path=native.sim_asset_file,object_scale=native.sim_asset_scale)
            env.reset(seed=native.seed)
            robot_base=bridge.get_robot_base_transform(env)
            if robot_base is None:raise RuntimeError('Native simulator did not expose robot base transform')
            calibration=np.load(calibration_path)
            report['robot_base']=robot_base.tolist();report['native_mapping_flags']={k:getattr(native,k)
                for k in ('use_direct_camera_extrinsic','no_map_foundationpose_through_robot_base',
                          'lock_object_z_to_table','min_object_center_z_margin')}
            results=[]
            for name,item in original['objects'].items():
                object_args=module._object_args_for_cache(native,bridge,name)
                local=np.eye(4);local[:3,:3]=bridge.euler2mat(*np.deg2rad(object_args.foundationpose_local_rotation_offset_deg))
                world=np.asarray(item['T_world_obj'],dtype=float)
                camera=inverse_native_affine_pose(world,calibration,robot_base,
                    object_args.foundationpose_position_offset,local,
                    direct_calibration=object_args.use_direct_camera_extrinsic,
                    map_robot_base=not object_args.no_map_foundationpose_through_robot_base)
                row={'object_name':name,'T_cam_obj':camera.tolist(),'ok':True,
                     'score':item.get('score',0.),'prompt':item.get('label',name),
                     'mask_source':'format_conversion_not_perception','sam3_instance_index':0}
                # Preserve the SAME original SAM6D orientation fix too.
                row=module._maybe_fix_bitong_mouth_orientation(object_args,name,row,calibration)
                mapped=bridge.map_camera_pose_to_pick_world(np.asarray(row['T_cam_obj']),calibration,env,object_args)
                delta=mapped-world
                equivalent=bool(np.max(np.abs(delta))<=1.e-6)
                report['objects'].append({'object_name':name,'equivalent':equivalent,
                    'translation_delta_m':delta[:3,3].tolist(),'max_matrix_delta':float(np.max(np.abs(delta)))})
                results.append(row)
            report['all_objects_equivalent']=all(row['equivalent'] for row in report['objects'])
            if report['all_objects_equivalent']:
                atomic_json(output/'fixed_sam6d_schema.json',{'results':results,
                    'source':'exact_native_mapping_roundtrip_of_existing_fixed_world_scene',
                    'perception_inference_verified':False,'source_sha256':report['source_sha256']})
    except BaseException as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
    finally:
        if env is not None:env.close()
        atomic_json(output/'result.json',report)
    print(json.dumps(report))
    return 0 if report['all_objects_equivalent'] else 42


if __name__=='__main__':raise SystemExit(main())
