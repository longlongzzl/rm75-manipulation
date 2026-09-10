from dataclasses import replace
from types import SimpleNamespace as NS
import numpy as np
import pytest

from rm75_app.planning.contracts import Pose,CollisionObject,PlanningScene
from rm75_app.pusht.model import Config,Push,rectangles,rotation
from rm75_app.pusht.motion import CuroboPushExecutor
from rm75_app.pusht.closed_gripper import ToolGeometry,bind_push,native_tool_geometry
from rm75_app.pusht.cartesian_ik import PushPathRejected
from rm75_app.pusht.observation import Observation


def fixture(yaw=0.):
    cfg=Config();obs=Observation('unit',1,1.,(.35,-.18,yaw),'simulation')
    # Fixed crossbar-side geometry case, independent of changing MPC rankings.
    _,bar_y,bw,_=rectangles(cfg)[0];r=rotation(yaw)
    push=Push(tuple(np.asarray(obs.pose[:2])+r@[-bw/2-cfg.pusher_radius_m,bar_y]),
              tuple(r@[1.,0.]),.012,cfg.speed_mps)
    profile=dict(tool_frame='gripper_tcp',push_tcp_z_m=.02,hover_clearance_m=.08,
                 tool_quaternion_wxyz=[1,0,0,0],pusher_contact_links=['right_pad'],
                 tool_collision_geometry_verified=True,object_centroid_z_m=.02,object_height_m=.04,
                 static_collision_objects=[dict(name='table',kind='cuboid',position=[.4,-.18,-.02],
                                                quaternion_wxyz=[1,0,0,0],dimensions=[.8,.8,.04])])
    # Radius/offset deliberately unlike the surrogate 5 mm circle.
    geometry=ToolGeometry(np.array([[.030,.008,0.,.012],[0.,0.,.10,.025]]),('right_pad','gripper_base_link'))
    scene=CuroboPushExecutor(None,None,cfg,profile,None,None,None)._scene(obs)
    return cfg,obs,push,profile,geometry,scene


def test_whole_chain_uses_closed_tool_contact_not_surrogate_circle_tcp():
    cfg,obs,push,p,g,scene=fixture();before=g.spheres.copy()
    points,e=bind_push(push,obs,cfg,p,g,scene)
    positions={name:xyz for name,xyz,_,_ in points}
    expected_surface=np.asarray(push.contact)+np.asarray(push.direction)*cfg.pusher_radius_m
    np.testing.assert_allclose(e['surface_contact_xyz'][:2],expected_surface)
    np.testing.assert_allclose(positions['contact'][:2],expected_surface-[.042,.008])
    np.testing.assert_allclose(positions['push']-positions['contact'],[.012,0,0])
    np.testing.assert_allclose(positions['retreat']-positions['push'],[-.01,0,.05])
    np.testing.assert_allclose(e['tcp_retreat_backoff_xyz'],positions['push']-np.r_[np.asarray(push.direction)*.01,0])
    assert e['retreat_collision_checks'] is False
    np.testing.assert_allclose(positions['approach']-positions['descend'],[0,0,.08])
    np.testing.assert_array_equal(g.spheres,before)
    assert e['nominal_audits'][0]['minimum_clearance_m']>0


def test_descend_accounts_for_a_wider_sphere_in_the_vertical_sweep():
    cfg,obs,push,p,g,scene=fixture()
    # This low sphere passes the target during descent but is below it at contact.
    g=ToolGeometry(np.vstack([g.spheres,[.06,0,-.05,.01]]),(*g.links,'right_pad'))
    # Table would block that geometry: leave a lower authored table for this UNIT test.
    table=CollisionObject('table','cuboid',Pose([.4,-.18,-.10],[1,0,0,0]),dimensions=[.8,.8,.04])
    scene=PlanningScene((*scene.objects[:-1],table))
    points,e=bind_push(push,obs,cfg,p,g,scene)
    assert e['descend_backoff_m']>.03


def test_neighbor_is_not_ignored_to_make_corrected_entry_pass():
    cfg,obs,push,p,g,scene=fixture();points,_=bind_push(push,obs,cfg,p,g,scene)
    entry=points[1][1]
    obstacle=CollisionObject('neighbor','cuboid',Pose(entry+[.03,.008,.04],[1,0,0,0]),dimensions=[.03,.03,.03])
    with pytest.raises(PushPathRejected,match='neighbor'):
        bind_push(push,obs,cfg,p,g,PlanningScene((*scene.objects,obstacle)))


def test_palm_leading_contact_is_rejected_not_exempted():
    cfg,obs,push,p,g,scene=fixture()
    g=ToolGeometry(np.array([[.030,.008,0.,.012]]),('gripper_base_link',))
    with pytest.raises(PushPathRejected,match='No allowed'):
        bind_push(push,obs,cfg,p,g,scene)


@pytest.mark.parametrize('spheres',[[],[[0,0,0,-.01]],[[0,0,float('nan'),.01]]])
def test_invalid_geometry_fails_closed(spheres):
    cfg,obs,push,p,g,scene=fixture();g=ToolGeometry(np.array(spheres),('right_pad',))
    with pytest.raises(PushPathRejected):bind_push(push,obs,cfg,p,g,scene)


def test_invalid_surface_intent_is_not_silently_moved_to_another_target():
    cfg,obs,push,p,g,scene=fixture();push=replace(push,contact=(.28,-.1))
    with pytest.raises(PushPathRejected,match='surface'):
        bind_push(push,obs,cfg,p,g,scene)


def test_open_native_geometry_never_qualifies_as_closed():
    backend=NS(_gripper_collision_state='open',config=NS(dynamic_gripper_collision=True))
    with pytest.raises(PushPathRejected,match='Closed native'):
        native_tool_geometry(backend,np.zeros(7))


def test_finite_crossbar_can_pass_beside_a_more_forward_high_sphere():
    cfg,obs,push,p,g,scene=fixture()
    g=ToolGeometry(np.vstack([g.spheres,[.060,.10,.025,.015]]),(*g.links,'right_pad'))
    _,row=bind_push(push,obs,cfg,p,g,scene)
    assert row['contact_features_tested']==3 and row['valid_contact_features']==1
    assert row['contact_sphere_index']==0 and row['descend_backoff_m']>.04


def test_workspace_checks_complete_low_tool_not_surrogate_radius():
    cfg,obs,push,p,g,scene=fixture()
    g=ToolGeometry(np.vstack([g.spheres,[-.20,0,0,.01]]),(*g.links,'gripper_base_link'))
    with pytest.raises(PushPathRejected,match='footprint_outside_workspace'):
        bind_push(push,obs,cfg,p,g,scene)


@pytest.mark.parametrize('yaw', [0., .25, -.4])
def test_retreat_backoff_follows_opposite_push_direction(yaw):
    cfg,obs,push,p,g,scene=fixture(yaw);points,e=bind_push(push,obs,cfg,p,g,scene)
    end=dict((name,xyz) for name,xyz,*_ in points)['push']
    np.testing.assert_allclose(np.asarray(e['tcp_retreat_backoff_xyz'])-end,
                               np.r_[-np.asarray(push.direction)*.01,0.])
    np.testing.assert_allclose(np.asarray(e['tcp_retreat_lift_xyz'])-e['tcp_retreat_backoff_xyz'],[0,0,.05])


@pytest.mark.parametrize('angle',[-.35,.35])
def test_oblique_push_preserves_surface_tangency_and_moves_along_requested_direction(angle):
    cfg,obs,push,p,g,scene=fixture()
    direction=rotation(angle)@np.asarray(push.direction)
    oblique=replace(push,direction=tuple(direction),normal=push.direction,length_m=.03)
    straight,_=bind_push(push,obs,cfg,p,g,scene)
    points,e=bind_push(oblique,obs,cfg,p,g,scene)
    points={name:xyz for name,xyz,*_ in points}
    np.testing.assert_allclose(points['contact'],dict((n,x) for n,x,*_ in straight)['contact'])
    np.testing.assert_allclose(points['push']-points['contact'],np.r_[direction*.03,0.])
    np.testing.assert_allclose(e['surface_contact_xyz'][:2],np.asarray(push.contact)+np.asarray(push.direction)*cfg.pusher_radius_m)
