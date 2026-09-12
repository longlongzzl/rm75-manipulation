"""Adapters for the existing SAM3D -> FoundationPose/RRTrack visual path.

Checkpoint capture is on demand; no realtime control thread is introduced and no
second tracker is implemented. Acquisition timestamps are never replaced by the
time a cached estimate is consumed. Missing/ambiguous instances stay invalid.
"""
from __future__ import annotations
import copy
import hashlib
from pathlib import Path
import numpy as np
from .scene import transform, ObservationUnavailable, positive, identifier


class RRTrackCheckpointSource:
    """Bridge existing RRTrackInstanceSample objects into one SWM observation.

    capture_samples(ids, after, boundary) must actively acquire/process a new
    burst with the SAME shared RRTrack/FoundationPose service. It returns
    {'samples': [...], 'sensor_session': str, 'robot': fresh_feedback}. Identity
    bindings and uncertainty budgets come from calibration, not the LLM.
    """
    def __init__(self, capture_samples, bindings, T_world_camera, calibration_id, *, domain='real'):
        self.capture_samples=capture_samples; self.bindings=copy.deepcopy(bindings)
        self.extrinsic=transform(T_world_camera);self.calibration_id=calibration_id;self.domain=domain

    def capture(self, ids, *, after, boundary):
        raw=self.capture_samples(ids, after=after, boundary=boundary)
        rows=[]
        for sample in raw['samples']:
            oid=sample.instance_id
            if oid not in self.bindings: raise ObservationUnavailable('Unregistered tracked instance')
            binding=self.bindings[oid];out=sample.output
            if sample.asset_name!=binding['asset_name']: raise ObservationUnavailable('Tracker instance/model mismatch')
            state=str(getattr(out.state,'value',out.state)).lower()
            if out.accepted is not True or state in ('lost','recovering') or out.T_cam_obj is None:
                raise ObservationUnavailable(f'{oid}: RRTrack output is not accepted')
            rows.append(dict(id=oid,mesh_sha256=binding['mesh_sha256'],
                T_world_object=(self.extrinsic@transform(out.T_cam_obj)).tolist(),
                captured_at=float(sample.timestamp_s),sequence=int(out.frame_index),
                accepted=True,tracking_state=state,source='rrtrack_foundationpose',
                position_uncertainty_m=binding['position_uncertainty_m'],
                rotation_uncertainty_rad=binding['rotation_uncertainty_rad']))
        return dict(schema='rm75_swm_observation_v1',world_frame='base_link',
                    calibration_id=self.calibration_id,domain=self.domain,
                    sensor_session=raw['sensor_session'],objects=rows,robot=raw['robot'])


class FoundationPoseCheckpointEstimator:
    """Optional on-demand update inside the existing pose service.

    `estimator` is the already loaded FoundationPose instance (metre-scale mesh).
    `quality_check` must independently validate rendering/depth/mask agreement.
    After a long gap or rejection, call register with the fresh instance mask;
    track_one is used only for sufficiently close accepted frames.
    """
    def __init__(self, estimator, quality_check, *, max_tracking_gap_s=.5):
        self.estimator=estimator;self.quality_check=quality_check
        self.max_gap=positive(max_tracking_gap_s,'max_tracking_gap_s');self.last_capture=None

    def estimate(self, *, rgb, depth_m, mask, K, captured_at):
        rgb=np.asarray(rgb);depth=np.asarray(depth_m);mask=np.asarray(mask);K=np.asarray(K)
        if (depth.ndim!=2 or mask.shape!=depth.shape or rgb.shape[:2]!=depth.shape
                or K.shape!=(3,3) or not np.isfinite(K).all() or not np.isfinite(captured_at)):
            raise ValueError('Invalid calibrated RGB-D checkpoint inputs')
        if not mask.astype(bool).any(): raise ObservationUnavailable('Target instance mask is empty')
        # Zero depth may represent missing pixels; nonfinite or negative depth is not valid geometry.
        if not np.isfinite(depth).all() or (depth<0).any(): raise ObservationUnavailable('Depth must be finite metres')
        if self.last_capture is not None and captured_at<=self.last_capture:
            raise ObservationUnavailable('Replayed frame')
        register=self.last_capture is None or captured_at-self.last_capture>self.max_gap
        try:
            if register:
                pose=self.estimator.register(K=K,rgb=rgb,depth=depth,ob_mask=mask.astype(bool),iteration=5)
            else:
                pose=self.estimator.track_one(rgb=rgb,depth=depth,K=K,iteration=2)
            pose=transform(pose)
            quality=self.quality_check(pose,rgb=rgb,depth_m=depth,mask=mask,K=K)
            if not isinstance(quality,dict) or quality.get('accepted') is not True:
                raise ObservationUnavailable('Pose failed independent rendering/depth agreement')
            self.last_capture=float(captured_at)
            return dict(T_cam_obj=pose.tolist(),captured_at=float(captured_at),
                        method='register' if register else 'track_one',**quality)
        except BaseException:
            self.last_capture=None
            raise


class SAM3DAssetBuilder:
    """Run the documented SAM3D inference API, then an explicit mesh exporter.

    Output dictionaries differ across SAM3D releases, so mesh_exporter is supplied
    by the installed SAM3D integration instead of guessing a gaussian PLY is a
    collision mesh. Metric transform is externally validated by depth/measurement.
    """
    def __init__(self, inference, mesh_exporter):
        self.inference=inference;self.mesh_exporter=mesh_exporter

    def build(self, *, image, mask, asset_id, output_dir, model_to_metric,
              scale_evidence, collision_builder, seed=42, pointmap=None):
        identifier(asset_id)
        if not isinstance(scale_evidence,str) or not scale_evidence:
            raise ValueError('Depth/physical scale evidence is mandatory')
        m=np.asarray(model_to_metric,dtype=float)
        if m.shape!=(4,4) or not np.isfinite(m).all() or not np.allclose(m[3],[0,0,0,1]):
            raise ValueError('Invalid calibrated model-to-metric affine transform')
        scales=np.linalg.norm(m[:3,:3],axis=0)
        if min(scales)<=0 or not np.allclose(scales,scales[0],rtol=1e-5):
            raise ValueError('Use a verified uniform metric scale, not shape-distorting fitting')
        proper=m.copy();proper[:3,:3]/=scales[0];transform(proper)
        destination=Path(output_dir).resolve()
        if destination.exists(): raise FileExistsError('Do not overwrite existing model assets')
        raw=self.inference(image,mask,seed=seed,pointmap=pointmap)
        mesh=self.mesh_exporter(raw)  # expected trimesh-like actual triangle mesh
        if not hasattr(mesh,'faces') or len(mesh.faces)==0:
            raise ValueError('SAM3D result has no mesh triangles; splats alone are not a physics shape')
        mesh=mesh.copy();mesh.apply_transform(m)
        proxy=collision_builder(mesh.copy())
        if not hasattr(proxy,'faces') or len(proxy.faces)==0 or not proxy.is_watertight or not proxy.is_convex:
            raise ValueError('Collision proxy must be watertight and convex with a validated volume')
        volume=float(proxy.volume)
        positive(volume,'collision_proxy_volume')
        destination.mkdir(parents=True)
        visual_path=destination/(asset_id+'.ply');collision_path=destination/(asset_id+'_collision.ply')
        mesh.export(visual_path);proxy.export(collision_path)
        return dict(source='sam3d',units='m',metric_scale_verified=True,
                    scale_evidence=scale_evidence,mesh_path=str(visual_path),
                    mesh_sha256=hashlib.sha256(visual_path.read_bytes()).hexdigest(),
                    collision_path=str(collision_path),
                    collision_sha256=hashlib.sha256(collision_path.read_bytes()).hexdigest(),
                    collision_kind='convex_mesh',volume_m3=volume,
                    volume_model='watertight_collision_proxy_not_measured_mass',
                    model_to_metric=m.tolist(),functional_poses=[])
