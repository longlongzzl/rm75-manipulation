"""Owned original PushT scene registration and measured SWM checkpoints.

No mirror acknowledgement or runtime admission is fabricated here.
"""
import hashlib
import time
import numpy as np
from rm75_app.swm.scene import SceneWorldModel,SyncPolicy,ObservationUnavailable,digest,transform
from rm75_app.swm.native_body_mirror import native_body,body_state,pose_matrix
from rm75_app.swm.native_bootstrap import _array
from rm75_app.workcell.io import atomic_json


class PushTSceneCapture:
    def __init__(self,session):
        import trimesh
        if not session.full_arm:raise ValueError('SWM capture requires the original articulated world')
        self.session=session;self.directory=session.directory/'swm';self.directory.mkdir(exist_ok=False)
        self.actors={'dynamic_T':session.base.target}
        for actor in session.base.world:
            if actor.name in self.actors:raise ValueError('Duplicate native scene identity')
            self.actors[actor.name]=actor
        assets={};objects=[]
        for oid,actor in self.actors.items():
            state=body_state(native_body(actor));parts=[];meshes=[]
            for shape in state['shapes']:
                if shape['kind']!='Box':raise ValueError('Original PushT box geometry required')
                dims=2*np.asarray(shape['geometry']['half_size']);local=shape['T_object_shape']
                parts.append(dict(dimensions_m=dims.tolist(),T_collision_part=local))
                mesh=trimesh.creation.box(extents=dims);mesh.apply_transform(local);meshes.append(mesh)
            if not parts:raise ValueError('Native scene actor has no collision geometry')
            mesh_path=self.directory/f'{oid}.ply';trimesh.util.concatenate(meshes).export(mesh_path)
            collision_path=self.directory/f'{oid}.json'
            atomic_json(collision_path,dict(schema='rm75_compound_cuboids_v1',units='m',parts=parts))
            atomic_json(self.directory/f'{oid}_native_body.json',state)
            volume=sum(float(np.prod(p['dimensions_m'])) for p in parts)
            assets[oid]=dict(source='cad',units='m',metric_scale_verified=True,
                scale_evidence='Original native PhysX box half sizes and local transforms',
                mesh_path=str(mesh_path),mesh_sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
                collision_path=str(collision_path),collision_sha256=hashlib.sha256(collision_path.read_bytes()).hexdigest(),
                collision_kind='compound_cuboids',collision_role='physical',volume_m3=volume,
                nominal_density_kg_m3=state['mass']/volume if state['kind']=='dynamic' else 1000.,
                functional_poses=[])
            objects.append(dict(id=oid,name=oid,asset_id=oid,fixed=state['kind']=='static'))
        self.calibration='pusht_native_base_'+digest(dict(urdf=session.report['urdf_sha256']))[:24]
        manifest=dict(schema='rm75_swm_v1',world_frame='base_link',observation_domain='physics',
            calibration_id=self.calibration,assets=assets,objects=objects)
        self.world=SceneWorldModel(manifest);self.sequence=0
        atomic_json(self.directory/'manifest.json',manifest)

    def _idle(self):
        base=self.session.base
        velocity=_array(base.robot.get_qvel()).reshape(-1)
        if velocity.shape!=(13,) or np.max(abs(velocity))>.001:return False
        if not self.session.stable():return False
        for link in base.tool_links:
            impulse=_array(base.scene.get_pairwise_contact_impulses(link,base.target))
            if float(np.linalg.norm(impulse))>0:return False
        return True

    def capture(self,boundary):
        session=self.session;session.stop.check()
        for _ in range(201):
            if self._idle():break
            session.advance(1/session.base.control_freq)
        else:raise ObservationUnavailable('Native PushT idle/no-holding checkpoint unavailable')
        self.world.check_assets()
        stamp=time.monotonic();clock=session.simulation_clock.read();base=session.base
        root=pose_matrix(base.robot.pose)
        if not np.allclose(root,np.eye(4),atol=1e-7,rtol=0):
            raise ObservationUnavailable('PushT base/world calibration changed')
        feedback=session.measured_tool_feedback();self.sequence+=1
        q=_array(base.robot.get_qpos()).reshape(-1)
        rows=[]
        for oid,actor in self.actors.items():
            body=native_body(actor);fixed=oid!='dynamic_T'
            matrix=transform(pose_matrix(actor.pose))
            linear=np.zeros(3) if fixed else _array(actor.linear_velocity).reshape(3)
            angular=np.zeros(3) if fixed else _array(actor.angular_velocity).reshape(3)
            rows.append(dict(id=oid,T_world_object=matrix.tolist(),
                mesh_sha256=self.world.assets[oid]['mesh_sha256'],captured_at=stamp,
                sequence=self.sequence,accepted=True,tracking_state='tracking',
                source='native_primary_PhysX_readback',position_uncertainty_m=0.,rotation_uncertainty_rad=0.,
                linear_velocity=linear.tolist(),angular_velocity=angular.tolist(),
                simulation_clock=clock,fixed_constraint_observed=fixed))
        if (session.simulation_clock.read()!=clock or not self._idle() or
                not np.array_equal(q,_array(base.robot.get_qpos()).reshape(-1))):
            raise ObservationUnavailable('Native state changed during PushT capture')
        robot=dict(joint_names=feedback['joint_names'],positions=feedback['joint_positions'],
            units='rad',idle=True,captured_at=stamp,T_world_tcp=feedback['T_world_tcp'],
            gripper_joint_positions=feedback['gripper_joint_positions'],gripper_observed=True,
            holding='empty',holding_evidence='Native target has no tool contact and no attachment in owned PushT environment',
            simulation_clock=clock)
        batch=dict(schema='rm75_swm_observation_v1',world_frame='base_link',domain='physics',
            calibration_id=self.calibration,sensor_session=session.session,objects=rows,robot=robot)
        prepared=self.world.prepare_checkpoint(batch,after=stamp-1e-6,now=time.monotonic(),
            policy=SyncPolicy(),boundary=boundary)
        snapshot=self.world.commit_checkpoint(prepared)
        atomic_json(self.directory/f'snapshot_{snapshot["revision"]:04d}.json',snapshot)
        session.events.emit('swm_native_push_checkpoint',boundary=boundary,
            snapshot_id=snapshot['snapshot_id'],revision=snapshot['revision'],
            object_ids=sorted(self.actors),mirror_acknowledged=False)
        return snapshot
