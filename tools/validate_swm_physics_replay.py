#!/usr/bin/env python3
"""Actual CPU PhysX subsystem evidence, NOT full-arm/three-task qualification."""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.swm.scene import SceneWorldModel, SyncPolicy, digest
from rm75_app.swm.physics_replay import SubprocessReplayWorld
from rm75_app.swm.identification import IsolatedReplayPool, infer_posterior, validate_transition


def pose(x=0., y=0., z=0.):
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix.tolist()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--python', default=sys.executable)
    parser.add_argument("--stationary", action="store_true")
    parser.add_argument("--lateral-offset-m", type=float, default=0.)
    parser.add_argument('--compound-pusht-profile',type=Path,
                        help='Use original two-box T geometry; still a tool-only reference experiment')
    args = parser.parse_args()
    if not np.isfinite(args.lateral_offset_m) or abs(args.lateral_offset_m) > .02:
        raise ValueError("Bounded synthetic tool offset required")
    from tools.run_network_isolated import block_network
    block_network()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    import trimesh

    assets = {}
    for name, dims in [('target', [.04, .04, .04]), ('table', [1., 1., .04]), ('obstacle', [.05, .05, .05])]:
        path = args.output / (name+'.ply')
        trimesh.creation.box(extents=dims).export(path)
        sha = hashlib.sha256(path.read_bytes()).hexdigest()
        assets[name] = dict(source='cad', units='m', metric_scale_verified=True,
            scale_evidence='analytic metric cuboid dimensions', mesh_path=str(path), mesh_sha256=sha,
            collision_path=str(path), collision_sha256=sha, collision_kind='cuboid',
            collision_dimensions_m=dims, volume_m3=float(np.prod(dims)), functional_poses=[])
    target_z=.0201
    if args.compound_pusht_profile is not None:
        from rm75_app.pusht.model import Config,rectangles
        profile=json.loads(args.compound_pusht_profile.read_text())['pusht']
        config=Config.from_dict(profile['model']);height=float(profile['physics']['motion']['object_height_m'])
        parts=[];meshes=[]
        for x,y,w,h in rectangles(config):
            dims=[w,h,height];local=pose(x,y,0.)
            parts.append(dict(dimensions_m=dims,T_collision_part=local))
            mesh=trimesh.creation.box(extents=dims);mesh.apply_transform(local);meshes.append(mesh)
        mesh_path=args.output/'target_compound.ply';trimesh.util.concatenate(meshes).export(mesh_path)
        collision_path=args.output/'target_compound.json'
        collision_path.write_text(json.dumps(dict(schema='rm75_compound_cuboids_v1',units='m',parts=parts)))
        assets['target'].update(mesh_path=str(mesh_path),mesh_sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
            collision_path=str(collision_path),collision_sha256=hashlib.sha256(collision_path.read_bytes()).hexdigest(),
            collision_kind='compound_cuboids',collision_role='physical',
            volume_m3=sum(float(np.prod(p['dimensions_m'])) for p in parts),
            scale_evidence='Original PushT rectangles and profile object height; not a convex hull')
        assets['target'].pop('collision_dimensions_m');target_z=height/2+.0001
    manifest = dict(schema='rm75_swm_v1', world_frame='base_link', calibration_id='analytic_cpu_world_v1',
        observation_domain='physics', assets=assets,
        objects=[dict(id=n, name=n, asset_id=n, fixed=n!='target') for n in assets])
    world = SceneWorldModel(manifest)
    initial_poses = dict(target=pose(z=target_z), table=pose(z=-.02), obstacle=pose(.3, .3, .025))
    tool_start = pose(-.07, y=args.lateral_offset_m, z=.02)
    batch = dict(schema='rm75_swm_observation_v1', world_frame='base_link',
        calibration_id=manifest['calibration_id'], sensor_session='cpu_physx_reference', domain='physics',
        objects=[dict(id=n, T_world_object=p, mesh_sha256=assets[n]['mesh_sha256'], captured_at=10., sequence=0,
            accepted=True, tracking_state='tracking', source='simulator_ground_truth',
            position_uncertainty_m=.0001, rotation_uncertainty_rad=.001) for n, p in initial_poses.items()],
        robot=dict(joint_names=[f'joint_{i}' for i in range(1, 8)], positions=[0.]*7, units='rad', idle=True,
            captured_at=10., T_world_tcp=tool_start, holding='empty', gripper_observed=True,
            holding_evidence='tool_only_simulation_no_gripper_or_articulated_arm'))
    # Initialization metadata is not task checkpoint proof. The reference
    # engine reads the initialized actors and its measured tool below.
    world.commit_checkpoint(world.prepare_checkpoint(batch, after=9., now=10., policy=SyncPolicy(), boundary='initialization'))
    snapshot = world.snapshot()
    command = dict(source='reference_command_program_only', time_s=[0., .5, .8, 1.1, 1.3, 1.6],
        T_world_tcp=[tool_start, tool_start, pose(-.04, y=args.lateral_offset_m, z=.02), pose(.02, y=args.lateral_offset_m, z=.02), pose(.02, y=args.lateral_offset_m, z=.08), pose(.02, y=args.lateral_offset_m, z=.08)],
        stages=['post_settle', 'approach', 'push', 'retreat', 'post_settle', 'post_settle'])
    if args.stationary:
        command["T_world_tcp"] = [copy.deepcopy(tool_start) for _ in command["time_s"]]
    truth = dict(static_friction=.6, dynamic_friction=.45, density_kg_m3=800.)
    reference_request = dict(hypothesis_id='reference', parameters=truth, initial_snapshot=snapshot,
        actual_action=command, action_digest=digest(command), transition_digest='reference_initialization',
        object_id='target', support_id='table', sample_times=[0., .4, .5, 1.6])
    def factory(request):
        return SubprocessReplayWorld(request, python=args.python, tool_spheres=[[0., 0., 0., .012]],
            directory=args.output, timeout_s=90)
    reference = factory(reference_request)
    try:
        observed = reference.replay()
    finally:
        reference.close()
    feedback = observed['measured_tool_feedback']
    times = np.asarray(feedback['time_s'])
    index = int(np.argmin(np.abs(times-.5)))
    start = float(times[index])
    action = dict(source='measured_feedback', time_s=(times[index:]-start).tolist(),
        T_world_tcp=feedback['T_world_tcp'][index:], stages=feedback['stages'][index:])
    initial = copy.deepcopy(snapshot)
    initial['checkpoint'] = 'before_push_offline_reference'
    initial['objects']['target']['measured']['T_world_object'] = observed['T_world_object'][2]
    for row in initial['objects'].values():
        row['measured']['captured_at'] = 10.+start
        row['measured']['sequence'] = 2
    initial['robot']['captured_at'] = 10.+start
    initial['robot']['T_world_tcp'] = action['T_world_tcp'][0]
    initial.pop('snapshot_id')
    initial['snapshot_id'] = digest(initial)
    settling_delta = float(np.linalg.norm(np.asarray(observed['T_world_object'][1])[:3, 3] -
                                         np.asarray(observed['T_world_object'][2])[:3, 3]))
    if settling_delta > .0002:
        raise RuntimeError('Reference target did not settle before the measured action')
    transition = validate_transition(dict(schema='rm75_measured_transition_v1', domain='physics', object_id='target',
        support_id='table', observation_source='simulator_ground_truth', calibration_id=manifest['calibration_id'],
        sensor_session='cpu_physx_reference', mesh_sha256=assets['target']['mesh_sha256'], initial_snapshot=initial,
        final_snapshot_id='offline_reference_endpoint', action_id='reference_measured_action', initially_settled=True,
        settling_evidence=dict(observed_position_delta_m=settling_delta, interval_s=.1), intervened=False, holding_changed=False,
        actual_action=action, object_time_s=[0., 1.6-start],
        T_world_object=[observed['T_world_object'][2], observed['T_world_object'][-1]],
        object_accepted=[True, True], object_sequences=[2, 3]))
    hypotheses = [dict(id='low_friction', parameters=dict(static_friction=.15, dynamic_friction=.1, density_kg_m3=800.)),
        dict(id='truth', parameters=truth),
        dict(id='high_friction', parameters=dict(static_friction=1.1, dynamic_friction=.95, density_kg_m3=800.)),
        dict(id='other_density', parameters=dict(static_friction=.6, dynamic_friction=.45, density_kg_m3=1600.))]
    checked, rollouts = IsolatedReplayPool(factory, workers=1).run(transition, hypotheses)
    posterior = infer_posterior(checked, hypotheses, rollouts)
    report = dict(scope='actual_CPU_PhysX_tool_only_not_full_arm_or_three_task_qualification',
        hardware_connected=False, model_inference_run=False, native_workers=1, ground_truth_parameters=truth,
        experiment=dict(stationary=args.stationary,lateral_offset_m=args.lateral_offset_m,
                        compound_pusht_profile=str(args.compound_pusht_profile) if args.compound_pusht_profile else None),
        measured_transition=checked, posterior=posterior, rollouts=rollouts,
        reference_command_digest=digest(command), measured_action_digest=checked['action_digest'],
        all_valid=all(r.get('valid') is True for r in rollouts))
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(dict(scope=report['scope'], all_valid=report['all_valid'], posterior=posterior)))
    return 0 if report['all_valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
