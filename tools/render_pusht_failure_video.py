#!/usr/bin/env python3
"""Render saved PushT GPU tool-envelope evidence, NOT an executed robot path.

No solver, physics stepping, device access or fixture tuning. The original
nominal TCP samples are displayed discretely, ending at the first collision.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from types import SimpleNamespace as NS

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.diagnose_pusht_tool_envelope import fixture_observation, place_spheres
from rm75_app.planning.contracts import Pose
from rm75_app.pusht.motion import CuroboPushExecutor
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.pickplace_clearance_audit import sphere_box_contacts


def validate_evidence(input_bytes, envelope, result):
    data = json.loads(input_bytes)
    config, push, observation = fixture_observation(data)
    summary = envelope['summary']
    for record in (summary, result):
        if (record.get('case') != data['case'] or record.get('fixture') != data['fixture']
                or any(record.get(key) is not False for key in
                       ('execute_real', 'hardware_connected', 'hardware_profile_qualified', 'complete_chain'))):
            raise ValueError('Expected matching failed, unqualified no-hardware fixture evidence')
    if (summary.get('input_sha256') != hashlib.sha256(input_bytes).hexdigest()
            or any(summary.get(key) is not True for key in
                   ('diagnostic_only', 'diagnostic_complete', 'cpu_gpu_masks_equal', 'state_unchanged'))
            or summary.get('arm_self_or_ik_qualified') is not False):
        raise ValueError('Missing verified original input/GPU evidence')
    if any(result.get(key) != data[key] for key in ('motion', 'config', 'push')):
        raise ValueError('GPU result does not describe the saved input')
    match = re.search(r'cartesian_ik_failed:pusht:descend: waypoint=(\d+)/(\d+);', result.get('error', ''))
    if not match or tuple(map(int, match.groups())) != (
            summary['first_collision_sample'], summary['nominal_descend_intervals']):
        raise ValueError('First envelope overlap must match the recorded GPU failure sample')
    local = np.asarray(envelope['local_tool_spheres'], dtype=float)
    links = envelope['links']
    if (local.shape != (len(links), 4) or not len(links) or not np.isfinite(local).all()
            or np.any(local[:, 3] <= 0) or len(links) != summary['native_tool_spheres']):
        raise ValueError('Invalid saved sphere geometry')
    count = max(1, int(np.ceil(data['motion']['hover_clearance_m'] / .005)))
    entry = np.asarray(push.contact) - np.asarray(push.direction) * config.approach_gap_m
    z = data['motion']['push_tcp_z_m']
    expected = np.array([[*entry, value, *data['motion']['tool_quaternion_wxyz']]
                         for value in np.linspace(z + data['motion']['hover_clearance_m'], z, count + 1)])
    poses = np.asarray(envelope['nominal_tcp_poses'], dtype=float)
    if poses.shape != expected.shape or not np.allclose(poses, expected, rtol=0, atol=1e-12):
        raise ValueError('Nominal samples differ from the original fixture')
    scene = CuroboPushExecutor(None, None, config, data['motion'], None, None, None)._scene(observation)
    if any(obj.kind != 'cuboid' for obj in scene.objects):
        raise ValueError('Only the original cuboid scene is supported')
    objects = [NS(name=obj.name, dims=obj.dimensions, pose=obj.pose.as_curobo_list()) for obj in scene.objects]
    spheres = [place_spheres(local, Pose(row[:3], row[3:])) for row in poses]
    contacts = [sphere_box_contacts(row, links, objects) for row in spheres]
    compact = [{'sample': i, 'pairs': [{key: pair[key] for key in ('link', 'obstacle', 'overlap_m')}
                                     for pair in pairs]} for i, pairs in enumerate(contacts) if pairs]
    if compact != summary['contacts'] or [i for i, pairs in enumerate(contacts) if pairs] != summary['collision_samples']:
        raise ValueError('Saved contacts differ from the original unmodified geometry')
    return scene, spheres, contacts


def display_samples(first_collision, total_samples):
    if type(first_collision) is not int or not 0 <= first_collision < total_samples:
        raise ValueError('Missing first failure sample')
    # Repeated stills ONLY: no interpolated/invented IK or joint trajectories.
    return [(i, 40 if i == first_collision else 8) for i in range(first_collision + 1)]


def corrected_evidence(input_bytes, envelope, result):
    """Same original input/geometry, new GPU-validated binding; nominal video only."""
    from rm75_app.pusht.closed_gripper import ToolGeometry, bind_push
    data=json.loads(input_bytes);config,push,observation=fixture_observation(data)
    if (envelope['summary']['input_sha256']!=hashlib.sha256(input_bytes).hexdigest()
            or any(envelope['summary'].get(key) is not True for key in
                   ('diagnostic_complete','cpu_gpu_masks_equal','state_unchanged'))
            or result.get('complete_chain') is not True or result.get('validation_success') is not True
            or result.get('error') or any(result.get(key) is not False for key in
                ('execute_real','hardware_connected','hardware_profile_qualified'))
            or any(result.get(key)!=data[key] for key in ('case','fixture','motion','config','push'))):
        raise ValueError('Corrected video requires the original input/geometry and a complete no-hardware GPU result')
    scene=CuroboPushExecutor(None,None,config,data['motion'],None,None,None)._scene(observation)
    points,binding=bind_push(push,observation,config,data['motion'],
        ToolGeometry(envelope['local_tool_spheres'],tuple(envelope['links'])),scene)
    recorded=[row for row in result['events'] if row.get('event')=='push_closed_gripper_binding']
    if len(recorded)!=1 or any(not np.allclose(binding[key],recorded[0][key],rtol=0,atol=1e-6)
            for key in ('tcp_contact_xyz','tcp_descend_xyz','surface_contact_xyz')):
        raise ValueError('Rendered geometry does not reproduce the saved GPU binding')
    xyz={name:p for name,p,_,_ in points};poses=[];labels=[]
    for name,start,end in (('DESCEND',xyz['approach'],xyz['descend']),('APPROACH CONTACT',xyz['descend'],xyz['contact'])):
        count=max(1,int(np.ceil(np.linalg.norm(end-start)/.005)))
        for index,p in enumerate(np.linspace(start,end,count+1)):
            poses.append([*p,*data['motion']['tool_quaternion_wxyz']]);labels.append(f'{name} {index}/{count}')
    spheres=[place_spheres(envelope['local_tool_spheres'],Pose(row[:3],row[3:])) for row in poses]
    objects=[NS(name=o.name,dims=o.dimensions,pose=o.pose.as_curobo_list()) for o in scene.objects]
    contacts=[[p for p in sphere_box_contacts(row,envelope['links'],objects) if p['overlap_m']>1e-9] for row in spheres]
    if any(contacts):raise ValueError('Corrected nominal contact still overlaps original world')
    return scene,spheres,contacts,poses,labels


def render(input_path, envelope_path, result_path, output, *, corrected=False):
    raw = input_path.read_bytes()
    envelope_raw, result_raw = envelope_path.read_bytes(), result_path.read_bytes()
    envelope, result = json.loads(envelope_raw), json.loads(result_raw)
    if corrected:
        original_scene,spheres,contacts,poses,labels=corrected_evidence(raw,envelope,result)
        envelope={**envelope,'nominal_tcp_poses':poses,'summary':dict(envelope['summary'],
            first_collision_sample=len(poses)-1,nominal_descend_intervals=len(poses)-1)}
    else:
        original_scene, spheres, contacts = validate_evidence(raw, envelope, result)
    output.mkdir(parents=True, exist_ok=False)
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw, ImageFont
    import sapien
    from mani_skill.utils.sapien_utils import look_at

    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    scene = engine.create_scene(sapien.SceneConfig())
    scene.set_ambient_light([.65, .65, .65])
    scene.add_directional_light([.3, .4, -1], [.7, .7, .7], shadow=True)
    for obj in original_scene.objects:
        builder = scene.create_actor_builder()
        material = renderer.create_material()
        material.set_base_color([.16, .7, .38, 1] if obj.name.startswith('pusht_target') else [.35, .39, .46, 1])
        builder.add_box_visual(half_size=np.asarray(obj.dimensions) / 2, material=material)
        actor = builder.build_static(name=obj.name)
        actor.set_pose(sapien.Pose(obj.pose.position, obj.pose.quaternion_wxyz))
    actors = []
    for index, row in enumerate(spheres[0]):
        pair = []
        for name, color in (('clear', [.18, .48, .95, 1]), ('overlap', [1, .08, .035, 1])):
            builder = scene.create_actor_builder()
            material = renderer.create_material()
            material.set_base_color(color)
            builder.add_sphere_visual(radius=float(row[3]), material=material)
            pair.append(builder.build_static(name=f'{name}_{index}'))
        actors.append(pair)
    center = np.mean([obj.pose.position for obj in original_scene.objects if obj.name.startswith('pusht_target')], axis=0)
    target = center + [-.025, 0, .06]
    cameras = []
    contact_view = np.asarray(envelope['nominal_tcp_poses'][-1][:3]) + [.015, 0, .015]
    for index, (aim, offset) in enumerate(((target, [-.34, -.24, .24]),
                                         (contact_view, [-.20, .18, .065]))):
        camera = scene.add_camera(f'evidence_{index}', 640, 416, .75, .005, 5)
        camera.set_pose(look_at(aim + offset, aim).sp)
        cameras.append(camera)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 18)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 16)
    summary = envelope['summary']
    frame_count, events = 0, []
    movie = output / ('pusht_corrected.mp4' if corrected else 'pusht_failure.mp4')
    with imageio.get_writer(movie, fps=10, codec='libx264', pixelformat='yuv420p', macro_block_size=8, quality=8) as writer:
        for index, repeat in display_samples(summary['first_collision_sample'], len(spheres)):
            colliding = {pair['sphere_index'] for pair in contacts[index]}
            for sphere_index, (row, pair) in enumerate(zip(spheres[index], actors)):
                for red, actor in enumerate(pair):
                    actor.set_pose(sapien.Pose(row[:3] if bool(red) == (sphere_index in colliding) else [0, 0, -50]))
            scene.update_render()  # NEVER scene.step, a solver or a device API.
            canvas = Image.new('RGB', (1280, 600), (17, 22, 30))
            for column, camera in enumerate(cameras):
                camera.take_picture()
                pixels = (np.clip(camera.get_picture('Color')[:, :, :3], 0, 1) * 255).astype(np.uint8)
                canvas.paste(Image.fromarray(pixels), (column * 640, 80))
            draw = ImageDraw.Draw(canvas)
            title='CORRECTED CLOSED-GRIPPER MAPPING' if corrected else 'HISTORICAL SYNTHETIC FIXTURE'
            draw.text((12, 8), f"PushT {summary['case']} | {title} | hardware profile NOT qualified", font=font, fill='white')
            draw.text((12, 34), 'GPU collision geometry at nominal TCP samples - NOT an IK / executed robot trajectory', font=font, fill='#ffd36e')
            draw.text((12, 60), 'Oblique view', font=small, fill='white')
            draw.text((652, 60), 'Side detail | Green: T | Blue: gripper spheres | Red: overlap', font=small, fill='white')
            overlap = max((pair['overlap_m'] for pair in contacts[index]), default=0) * 1000
            state = f'FIRST OVERLAP / recorded IK FAIL: max {overlap:.6f} mm' if colliding else 'No tool/world overlap at this nominal sample; arm/self/IK NOT certified'
            status=(f"{labels[index]} | Clear nominal geometry; source GPU five-stage chain PASS"
                    if corrected else f"Descend sample {index}/{summary['nominal_descend_intervals']} | {state}")
            draw.text((12, 504), status, font=font, fill='#ffb39c' if colliding else 'white')
            pairs = ', '.join(sorted({pair['link'] + ' <-> ' + pair['obstacle'] for pair in contacts[index]}))
            draw.text((12, 532), pairs or 'Original target/table dimensions, tool orientation, sphere centers and radii are unchanged.', font=small, fill='white')
            footer=('Nominal mapped samples, NOT joint-path or physics replay. Stop at first contact; no pretend T motion.' if corrected else
                    'Discrete saved positions, slowed for inspection. Stop at first collision. No physics / new GPU planning / hardware motion.')
            draw.text((12, 562), footer, font=small, fill='#ffd36e')
            for _ in range(repeat):
                writer.append_data(np.asarray(canvas))
            events.append({'sample': index, 'first_frame': frame_count, 'frames': repeat})
            frame_count += repeat
            if index in (0, summary['first_collision_sample']):
                canvas.save(output / f'sample_{index:02d}.jpg')
    metadata = dict(schema='rm75.pusht_failure_video/v1', case=summary['case'],
                    diagnostic_only=True, execute_real=False, hardware_connected=False,
                    hardware_profile_qualified=False, complete_chain=False, physics_stepped=False,
                    new_gpu_planning=False, representation='saved_nominal_tcp_tool_envelope_NOT_executed_trajectory',
                    frames=frame_count, fps=10, duration_s=frame_count / 10, events=events,
                    source_sha256={name: hashlib.sha256(value).hexdigest() for name, value in
                                   (('input', raw), ('envelope', envelope_raw), ('result', result_raw))},
                    video_sha256=hashlib.sha256(movie.read_bytes()).hexdigest())
    if corrected:
        metadata.update(representation='corrected_nominal_closed_tool_mapping_NOT_joint_path_or_physics',
                        source_gpu_complete_chain=True,video_ends_at_first_contact=True)
    atomic_json(output / 'recording.json', metadata)
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--envelope', type=Path, required=True)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--corrected', action='store_true',help='Compare original saved geometry with a new complete GPU binding')
    args = parser.parse_args()
    print(json.dumps(render(args.input, args.envelope, args.result, args.output,corrected=args.corrected)))


if __name__ == '__main__':
    main()
