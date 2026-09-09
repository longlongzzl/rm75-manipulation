#!/usr/bin/env python3
"""Step a dynamic T in ManiSkill using the saved GPU joint-path-driven full tool.

No robot/SDK, no replanning, no target pose prediction used to move the target.
This isolates tool contact dynamics; it does NOT simulate whole-arm actuation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.pusht.physics_replay import TcpFK, TimedProgram, audit_endpoints, measured_outcome
from rm75_app.pusht.model import predict
from rm75_app.workcell.io import atomic_json
from tools.diagnose_pusht_tool_envelope import fixture_observation
from tools.render_pusht_failure_video import corrected_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ('input', 'envelope', 'result', 'output'):
        parser.add_argument('--'+flag, type=Path, required=True)
    parser.add_argument('--stationary-control', action='store_true')
    args = parser.parse_args(); output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    report = dict(schema='rm75.pusht_maniskill_contact/v1', execute_real=False, hardware_connected=False,
        hardware_profile_qualified=False, verified_physical_success=None, arm_servo_simulated=False,
        physics_stepped=False, dynamics_completed=False, stationary_control=args.stationary_control,
        interpretation='Full original sphere tool driven kinematically along GPU joint-path FK; T moves only through PhysX',
        assumed_physics=dict(static_friction=.3, dynamic_friction=.3, restitution=0., density_kg_m3=1000.,
                             sim_freq=240, control_freq=30, hardware_calibrated=False))
    started = time.monotonic(); env = None; writer = None; samples = []; frames = 0
    try:
        raw = args.input.read_bytes(); data = json.loads(raw)
        envelope = json.loads(args.envelope.read_bytes()); result = json.loads(args.result.read_bytes())
        scene, *_ = corrected_evidence(raw, envelope, result)
        config, push, observation = fixture_observation(data)
        report.update(case=data['case'], fixture=data['fixture'],
            source_sha256={key:hashlib.sha256(getattr(args,key).read_bytes()).hexdigest()
                           for key in ('input', 'envelope', 'result')})
        urdf = ROOT/'assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf'
        # Exactly the same URDF bytes used by the existing GPU configuration.
        import yaml
        original = Path(yaml.safe_load(Path(result['planner_config']['robot_config']).read_text())['robot_cfg']['kinematics']['urdf_path'])
        if urdf.read_bytes() != original.read_bytes(): raise ValueError('Local and GPU URDF differ')
        report['urdf_sha256'] = hashlib.sha256(urdf.read_bytes()).hexdigest()
        program = TimedProgram(result, TcpFK(urdf))
        from rm75_app.pusht.closed_gripper import ToolGeometry, bind_push
        from transforms3d.quaternions import quat2mat
        points, _ = bind_push(push, observation, config, data['motion'],
            ToolGeometry(envelope['local_tool_spheres'], tuple(envelope['links'])), scene)
        report['fk_endpoint_audit'] = audit_endpoints(program, points,
            quat2mat(data['motion']['tool_quaternion_wxyz']),
            data['motion']['corridor_tolerance_m'], data['motion']['orientation_tolerance_rad'])
        report['implementation_sha256'] = {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            for name in ('rm75_app/pusht/physics_replay.py',
                         'rm75_app/simulation/pusht_contact_env.py',
                         'tools/run_pusht_maniskill_contact.py')}
        report['original_trajectory_duration_s'] = float(program.duration)
        report['tool_spheres'] = len(envelope['local_tool_spheres'])
        import gymnasium as gym
        import rm75_app.simulation.pusht_contact_env
        import imageio.v2 as imageio
        from PIL import Image, ImageDraw
        env = gym.make('RM75-PushT-Contact-v1', program=program, spheres=envelope['local_tool_spheres'],
            config=config, observation=observation, motion=data['motion'],
            static_objects=[o for o in scene.objects if not o.name.startswith('pusht_target_')],
            stationary=args.stationary_control, obs_mode='none', reward_mode='none',
            render_mode='rgb_array', sim_backend='physx_cpu', num_envs=1)
        env.reset(seed=0); base = env.unwrapped
        def observed():
            transform = base.target.pose.to_transformation_matrix().detach().cpu().numpy().reshape(4, 4)
            return (np.array([*transform[:2, 3], np.arctan2(transform[1, 0], transform[0, 0])]),
                    float(transform[2, 3]), float(np.arccos(np.clip(transform[2, 2], -1, 1))))
        initial, initial_z, _ = observed(); initial_settled = None
        writer = imageio.get_writer(output/'contact_physics.mp4', fps=10, codec='libx264', pixelformat='yuv420p')
        for step in range(int(np.ceil((program.duration+4)*base.control_freq))):
            env.step(None); report['physics_stepped'] = True
            pose, z, tilt = observed()
            if not np.isfinite([*pose, z, tilt]).all(): raise ValueError('Nonfinite physical target state')
            if initial_settled is None and base.physics_time >= base.settle_s: initial_settled = pose.copy()
            samples.append(dict(time_s=base.physics_time, stage=base.stage, pose=pose.tolist(), z_m=z,
                                tilt_rad=tilt))
            if step%3 == 0:
                pixels = env.render()
                if hasattr(pixels, 'detach'): pixels = pixels.detach().cpu().numpy()
                pixels = np.asarray(pixels)
                while pixels.ndim > 3: pixels = pixels[0]
                canvas = Image.new('RGB', (pixels.shape[1], pixels.shape[0]+64), (17, 22, 30))
                canvas.paste(Image.fromarray(pixels[..., :3].astype(np.uint8)), (0, 64))
                draw = ImageDraw.Draw(canvas)
                draw.text((8, 5), 'MANISKILL CONTACT PHYSICS | dynamic T | kinematic FULL closed-tool spheres', fill='white')
                draw.text((8, 23), f"{data['case']} | {base.stage} | t={base.physics_time:.2f}s | dx={(pose[0]-initial[0])*1000:.2f} mm | dy={(pose[1]-initial[1])*1000:.2f} mm", fill=(255, 215, 100))
                draw.text((8, 41), 'Assumed material; NO arm servo / hardware qualification. Original GPU joint path + timing.', fill='white')
                writer.append_data(np.asarray(canvas)); frames += 1
        final, final_z, final_tilt = observed(); goal = (.38, -.18, observation.pose[2])
        prediction = predict(observation.pose, push, config)
        report.update(dynamics_completed=True, simulated_time_s=base.physics_time,
            observed=measured_outcome(initial_settled, final, push, goal, config),
            initial_pose=initial.tolist(), settled_pose=initial_settled.tolist(), final_pose=final.tolist(),
            target_z_change_m=final_z-initial_z, surrogate_prediction=prediction.tolist(),
            final_tilt_rad=final_tilt, max_observed_tilt_rad=max(s['tilt_rad'] for s in samples),
            max_observed_z_change_m=max(abs(s['z_m']-initial_z) for s in samples),
            measurement_hz=base.control_freq,
            original_goal=list(goal),
            original_position_tolerance_m=config.position_tolerance_m,
            original_yaw_tolerance_rad=config.yaw_tolerance_rad,
            prediction_is_original_virtual_pusher_surrogate=True,
            prediction_position_error_m=float(np.linalg.norm(final[:2]-prediction[:2])),
            prediction_yaw_error_rad=float(abs((final[2]-prediction[2]+np.pi)%(2*np.pi)-np.pi)),
            target_contact_substeps=len(base.contacts), early_target_contact_substeps=base.early_contact_steps,
            tool_static_obstacle_contact_substeps=base.obstacle_contact_steps,
            contact_policy_passed=not (base.early_contact_steps or base.obstacle_contact_steps),
            first_target_contact=base.contacts[0] if base.contacts else None,
            contact_substeps_by_stage={name:sum(row['stage']==name for row in base.contacts)
                for name in ('settle', 'approach', 'descend', 'contact', 'push', 'retreat', 'post_settle')},
            target_kinematic=False, target_pose_updated_during_replay=False)
        atomic_json(output/'contacts.json', base.contacts)
    except Exception as exc:
        import traceback
        report.update(error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
    finally:
        if writer is not None: writer.close()
        if env is not None: env.close()
        report.update(elapsed_s=time.monotonic()-started, video_frames=frames)
        atomic_json(output/'observations.json', samples)
        atomic_json(output/'result.json', report)
    print(json.dumps({k:report.get(k) for k in ('case','dynamics_completed','error','observed','elapsed_s')}))
    return 0 if report['dynamics_completed'] else 42


if __name__=='__main__': raise SystemExit(main())
