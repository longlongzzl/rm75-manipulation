"""Renderer input contracts; CPU-only fixtures are NOT new GPU evidence."""
import copy
from dataclasses import asdict
import hashlib
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rm75_app.planning.contracts import Pose
from rm75_app.pusht.model import Config, choose_push
from rm75_app.pusht.motion import CuroboPushExecutor
from rm75_app.workcell.pickplace_clearance_audit import sphere_box_contacts
from tools.diagnose_pusht_tool_envelope import fixture_observation, local_spheres, place_spheres
from tools.render_pusht_failure_video import display_samples, validate_evidence,corrected_evidence


def unit_evidence():
    config = Config()
    push, _ = choose_push((.35, -.18, 0), (.38, -.18, 0), config)
    motion = dict(tool_frame='gripper_tcp', push_tcp_z_m=.02, hover_clearance_m=.08,
                  tool_quaternion_wxyz=[0, 0, 1, 0], pusher_contact_links=['right_pad'],
                  tool_collision_geometry_verified=True, object_centroid_z_m=.01,
                  object_height_m=.04, static_collision_objects=[dict(name='simulation_table',
                  kind='cuboid', position=[.4, -.18, -.03], quaternion_wxyz=[1, 0, 0, 0],
                  dimensions=[.4, .28, .04])])
    data = dict(scenario='authored_simulation_fixture', case='baseline', fixture='low_table',
                execute_real=False, hardware_connected=False, hardware_profile_qualified=False,
                config=asdict(config), push=push.as_dict(), motion=motion)
    raw = json.dumps(data).encode()
    _, _, observation = fixture_observation(data)
    scene = CuroboPushExecutor(None, None, config, motion, None, None, None)._scene(observation)
    entry = np.asarray(push.contact) - np.asarray(push.direction) * config.approach_gap_m
    poses = [[*entry, z, *motion['tool_quaternion_wxyz']] for z in np.linspace(.1, .02, 17)]
    local = local_spheres(np.array([[entry[0] + .016, entry[1], .1, .006]]), Pose(poses[0][:3], poses[0][3:]))
    links = ['right_pad']
    objects = [NS(name=obj.name, dims=obj.dimensions, pose=obj.pose.as_curobo_list()) for obj in scene.objects]
    pairs = [sphere_box_contacts(place_spheres(local, Pose(p[:3], p[3:])), links, objects) for p in poses]
    collisions = [i for i, row in enumerate(pairs) if row]
    summary = dict(case='baseline', fixture='low_table', diagnostic_only=True, diagnostic_complete=True,
                   cpu_gpu_masks_equal=True, state_unchanged=True, arm_self_or_ik_qualified=False,
                   execute_real=False, hardware_connected=False, hardware_profile_qualified=False,
                   complete_chain=False, input_sha256=hashlib.sha256(raw).hexdigest(),
                   first_collision_sample=collisions[0], collision_samples=collisions,
                   nominal_descend_intervals=16, native_tool_spheres=1,
                   contacts=[dict(sample=i, pairs=[{k: p[k] for k in ('link', 'obstacle', 'overlap_m')}
                                                  for p in row]) for i, row in enumerate(pairs) if row])
    envelope = dict(summary=summary, local_tool_spheres=local.tolist(), links=links, nominal_tcp_poses=poses)
    result = dict(json.loads(raw), complete_chain=False,
                  error=f'PushPathRejected: cartesian_ik_failed:pusht:descend: waypoint={collisions[0]}/16; ik_variants=0;')
    return raw, envelope, result


def test_video_uses_original_scene_geometry_without_mutating_saved_evidence():
    raw, envelope, result = unit_evidence()
    before = copy.deepcopy(envelope)
    scene, spheres, contacts = validate_evidence(raw, envelope, result)
    assert len(scene.objects) == 3 and len(spheres) == 17
    assert envelope == before and contacts[envelope['summary']['first_collision_sample']]
    np.testing.assert_array_equal(spheres[0][:, 3], np.array(envelope['local_tool_spheres'])[:, 3])


@pytest.mark.parametrize('key,value', [('execute_real', True), ('hardware_connected', True),
                                       ('scenario', 'live')])
def test_video_rejects_hardware_or_live_inputs(key, value):
    raw, envelope, result = unit_evidence()
    data = json.loads(raw)
    data[key] = value
    with pytest.raises(ValueError):
        validate_evidence(json.dumps(data).encode(), envelope, result)


@pytest.mark.parametrize('key,value', [('input_sha256', 'changed'), ('cpu_gpu_masks_equal', False),
                                       ('diagnostic_complete', False), ('hardware_profile_qualified', True)])
def test_video_rejects_unverified_or_mismatched_gpu_evidence(key, value):
    raw, envelope, result = unit_evidence()
    envelope['summary'][key] = value
    with pytest.raises(ValueError):
        validate_evidence(raw, envelope, result)


@pytest.mark.parametrize('change', ['sphere_radius', 'tcp', 'result_motion', 'failure_index', 'contacts'])
def test_video_rejects_modified_geometry_nominal_path_or_failure_claim(change):
    raw, envelope, result = unit_evidence()
    if change == 'sphere_radius':
        envelope['local_tool_spheres'][0][3] *= .5
    elif change == 'tcp':
        envelope['nominal_tcp_poses'][1][2] += .001
    elif change == 'result_motion':
        result['motion']['hover_clearance_m'] += .001
    elif change == 'failure_index':
        result['error'] = 'cartesian_ik_failed:pusht:descend: waypoint=1/16;'
    else:
        envelope['summary']['contacts'] = []
    with pytest.raises(ValueError):
        validate_evidence(raw, envelope, result)


def test_discrete_replay_stops_at_first_failure_and_only_holds_saved_samples():
    samples = display_samples(10, 17)
    assert samples == [(i, 8) for i in range(10)] + [(10, 40)]
    for invalid in (-1, 17, None, True):
        with pytest.raises(ValueError):
            display_samples(invalid, 17)


def corrected_unit_evidence():
    from rm75_app.pusht.closed_gripper import bind_push,ToolGeometry
    raw,envelope,result=unit_evidence();data=json.loads(raw)
    cfg,push,obs=fixture_observation(data)
    scene=CuroboPushExecutor(None,None,cfg,data['motion'],None,None,None)._scene(obs)
    _,binding=bind_push(push,obs,cfg,data['motion'],
        ToolGeometry(envelope['local_tool_spheres'],tuple(envelope['links'])),scene)
    result.update(complete_chain=True,validation_success=True,error=None,
                  events=[dict(event='push_closed_gripper_binding',**binding)])
    return raw,envelope,result


def test_corrected_video_stops_at_bound_contact_and_preserves_original_geometry():
    raw,envelope,result=corrected_unit_evidence();before=copy.deepcopy(envelope)
    scene,spheres,contacts,poses,labels=corrected_evidence(raw,envelope,result)
    assert envelope==before and not any(contacts) and len(scene.objects)==3
    np.testing.assert_allclose(poses[-1][:3],result['events'][0]['tcp_contact_xyz'])
    assert labels[0].startswith('DESCEND') and labels[-1].startswith('APPROACH CONTACT')


@pytest.mark.parametrize('change',['failed','hardware','binding','source_geometry'])
def test_corrected_video_cannot_promote_failed_or_mismatched_result(change):
    raw,envelope,result=corrected_unit_evidence()
    if change=='failed':result['validation_success']=False
    elif change=='hardware':result['hardware_connected']=True
    elif change=='binding':result['events'][0]['tcp_contact_xyz'][0]+=.01
    else:envelope['summary']['cpu_gpu_masks_equal']=False
    with pytest.raises(ValueError):corrected_evidence(raw,envelope,result)
