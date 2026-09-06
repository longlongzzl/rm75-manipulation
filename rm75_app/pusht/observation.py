"""Freshness-checked observations and calibrated AprilTag/RealSense acquisition."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import time
import uuid
import numpy as np
from rm75_app.workcell.io import read_json, integer, finite
from rm75_app.workcell.transforms import rigid, vector


@dataclass(frozen=True)
class Observation:
    session_id: str
    sequence: int
    captured_at: float
    pose: tuple[float,float,float]
    source: str
    frame: str = 'base_link'
    confidence: float = 1.

    @classmethod
    def from_dict(cls,payload):
        if payload.get('schema') != 'rm75_pusht_observation_v1':
            raise ValueError('Unsupported observation schema')
        pose=vector(payload.get('pose'),3,'pose')
        session=payload.get('session_id')
        if not isinstance(session,str) or not session:
            raise ValueError('Observation requires capture-session identity')
        return cls(session,integer(payload.get('sequence'),'sequence',0,2**63-1),
                   finite(payload.get('captured_at'),'captured_at',1),tuple(pose),
                   str(payload.get('source','')),str(payload.get('frame','')),
                   finite(payload.get('confidence',1),'confidence',0,1))

    def validate(self, *, now=None, after=0., previous=None,max_age_s=1.5, real=False):
        now=time.time() if now is None else now
        if self.frame != 'base_link':
            raise ValueError('Observation must be transformed into base_link, not camera/builder coordinates')
        if not after < self.captured_at <= now+.25 or now-self.captured_at > max_age_s:
            raise ValueError('stale_or_future_observation')
        if self.confidence < .8:
            raise ValueError('low_confidence_observation')
        if previous is not None and (self.session_id != previous.session_id or self.sequence <= previous.sequence):
            raise ValueError('replayed_or_changed_capture_session')
        if real and self.source not in ('realsense_apriltag','live_tracker'):
            raise ValueError('Real execution requires a live observation source')
        vector(self.pose,3,'pose')
        return self

    def as_dict(self):
        return dict(schema='rm75_pusht_observation_v1', session_id=self.session_id,
                    sequence=self.sequence,captured_at=self.captured_at,pose=list(self.pose),
                    source=self.source,frame=self.frame,confidence=self.confidence)


class JsonObserver:
    """For an existing live tracker that atomically writes the documented schema.

    Writer timestamps must be exposure/acquisition time on this machine, NOT file
    mtime or the time a stale pose was republished. A session restart ends the run.
    """
    def __init__(self,path,stop,timeout_s=4):
        self.path=Path(path);self.stop=stop;self.timeout_s=timeout_s

    def observe(self,after=0.):
        end=time.monotonic()+self.timeout_s
        last='no_observation'
        while time.monotonic()<end:
            self.stop.check()
            try:
                obs=Observation.from_dict(read_json(self.path))
                if obs.captured_at > after:
                    return obs
                last='stale_observation'
            except (OSError,ValueError,KeyError) as exc:
                last=str(exc)
            self.stop.wait(.025)
        raise TimeoutError(last)

    def close(self):
        pass


class AprilTagObserver:
    """RealSense color-frame AprilTag pose, explicit marker-to-object calibration.

    Does not assume the marker center equals the T area centroid. Does not connect
    any robot. Dependencies are loaded only on entering this live camera adapter.
    """
    def __init__(self,config,stop):
        import cv2
        import pyrealsense2 as rs
        if not hasattr(cv2,'aruco'):
            raise RuntimeError('OpenCV aruco/AprilTag support is missing')
        self.cv2=cv2;self.rs=rs;self.stop=stop
        self.T_base_camera=rigid(config['T_base_camera'],'T_base_camera')
        self.T_marker_object=rigid(config['T_marker_object'],'T_marker_object')
        self.marker_id=integer(config['marker_id'],'marker_id',0,586)
        self.marker_size=finite(config['marker_size_m'],'marker_size_m',.005,.2)
        self.max_reprojection=finite(config.get('max_reprojection_px',2),'max_reprojection_px',.1,5)
        self.max_tilt=finite(config.get('max_tilt_rad',.15),'max_tilt_rad',.01,.3)
        self.object_z=finite(config['object_centroid_z_m'],'object_centroid_z_m',-1,2)
        self.max_z_error=finite(config.get('max_z_error_m',.01),'max_z_error_m',.001,.03)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters())
        self.pipeline=rs.pipeline();cfg=rs.config()
        if config.get('serial'):
            cfg.enable_device(str(config['serial']))
        cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
        profile=self.pipeline.start(cfg)
        try:
            for sensor in profile.get_device().query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1.)
            self.timestamp_uncertainty=finite(config.get('timestamp_uncertainty_s',.02),'timestamp_uncertainty_s',.001,.1)
            self.max_exposure=finite(config.get('max_exposure_s',.05),'max_exposure_s',.001,.1)
            intr=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]])
            self.dist=np.asarray(intr.coeffs,dtype=float)
            self.session_id=uuid.uuid4().hex;self.sequence=0
        except BaseException:
            self.pipeline.stop()
            raise

    def observe(self,after=0.):
        deadline=time.monotonic()+4.
        cv2=self.cv2
        s=self.marker_size/2
        points=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],dtype=float)
        while time.monotonic()<deadline:
            self.stop.check()
            frames=self.pipeline.wait_for_frames(1000)
            frame=frames.get_color_frame()
            if not frame:
                continue
            # Use camera timestamps in the host clock domain. Reading a frame
            # now does NOT make a buffered old frame a fresh observation.
            if frame.get_frame_timestamp_domain() != self.rs.timestamp_domain.global_time:
                continue
            exposure=self.max_exposure
            if frame.supports_frame_metadata(self.rs.frame_metadata_value.actual_exposure):
                exposure=float(frame.get_frame_metadata(self.rs.frame_metadata_value.actual_exposure))/1e6
            captured=float(frame.get_timestamp())/1000.-exposure-self.timestamp_uncertainty
            if not after < captured <= time.time()+.25 or time.time()-captured>1.5:
                continue
            image=np.asanyarray(frame.get_data())
            corners,ids,_=self.detector.detectMarkers(image)
            if ids is None:
                continue
            indexes=np.flatnonzero(ids.reshape(-1)==self.marker_id)
            if len(indexes)!=1:
                continue
            pixel=np.asarray(corners[int(indexes[0])],dtype=float).reshape(4,2)
            ok,rvec,tvec=cv2.solvePnP(points,pixel,self.K,self.dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or float(tvec[2,0])<=0:
                continue
            projected,_=cv2.projectPoints(points,rvec,tvec,self.K,self.dist)
            residual=float(np.linalg.norm(projected.reshape(4,2)-pixel,axis=1).max())
            if residual > self.max_reprojection:
                continue
            T=np.eye(4);T[:3,:3]=cv2.Rodrigues(rvec)[0];T[:3,3]=tvec.reshape(3)
            T=self.T_base_camera@T@self.T_marker_object
            tilt=float(np.arccos(np.clip(T[2,2],-1,1)))
            if tilt>self.max_tilt or abs(T[2,3]-self.object_z)>self.max_z_error:
                continue
            self.sequence+=1
            return Observation(self.session_id,self.sequence,captured,
                               (float(T[0,3]),float(T[1,3]),float(np.arctan2(T[1,0],T[0,0]))),
                               'realsense_apriltag')
        raise TimeoutError('No valid calibrated AprilTag observation')

    def close(self):
        self.pipeline.stop()
