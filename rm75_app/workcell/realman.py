"""Explicit RealMan API2 adapter. SDK calls are in degrees, application in radians.

No connection at import time. No implicit homing, enabling, gripper opening or
error clearing. A failed command/feedback or cancellation latches the adapter.
"""
from __future__ import annotations
import time
import threading
import numpy as np
from .io import finite, integer
from .transforms import vector


class RealManArm:
    def __init__(self,config,stop,events,*,sdk=None):
        if config.get('hardware_reviewed') is not True or not config.get('qualification_id'):
            raise PermissionError('Hardware profile has not been locally qualified')
        self.config=config;self.stop_token=stop;self.events=events
        self.faulted=False;self._lock=threading.RLock();self.connected=False
        self.lower=vector(config['joint_lower_rad'],7,'joint_lower_rad')
        self.upper=vector(config['joint_upper_rad'],7,'joint_upper_rad')
        if not (self.lower<self.upper).all():
            raise ValueError('Invalid joint limits')
        self.start_gap=finite(config.get('max_start_gap_rad',.05),'max_start_gap_rad',.001,.1)
        self.following_error=finite(config.get('max_following_error_rad',.12),'max_following_error_rad',.01,.25)
        self.endpoint_error=finite(config.get('endpoint_error_rad',.025),'endpoint_error_rad',.001,.05)
        self.hz=finite(config.get('stream_hz',30),'stream_hz',10,100)
        self.max_jitter=finite(config.get('max_stream_lateness_s',.08),'max_stream_lateness_s',.01,.2)
        if sdk is None:
            from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
            sdk=RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.sdk=sdk
        handle=sdk.rm_create_robot_arm(str(config['ip']),integer(config.get('port',8080),'port',1,65535))
        if getattr(handle,'id',-1)<=0:
            raise ConnectionError('RealMan refused connection')
        self.connected=True
        try:
            code,mode=self.sdk.rm_get_arm_run_mode()
            self._ok(code,'get_arm_run_mode')
            if mode!=1:
                raise PermissionError('Controller is not in real mode; never enable it automatically')
            self.read_joints()
        except BaseException:
            self.close();raise

    def _ok(self,code,operation):
        if type(code) is not int or code!=0:
            self.faulted=True
            raise RuntimeError(f'RealMan {operation} failed: {code!r}')

    def read_joints(self):
        with self._lock:
            code,value=self.sdk.rm_get_joint_degree()
            self._ok(code,'read_joints')
            result=np.deg2rad(vector(value,7,'RealMan joint feedback (degrees)'))
            if np.any(result<self.lower-.001) or np.any(result>self.upper+.001):
                self.faulted=True
                raise RuntimeError('Measured joints outside qualified limits')
            return result

    def controlled_stop(self):
        self.faulted=True
        with self._lock:
            if self.connected:
                code=self.sdk.rm_set_arm_stop()
                self.events.emit('controlled_stop',return_code=code,physical_estop=False)
                self._ok(code,'stop')

    def execute(self,positions,times,*,stage):
        path=np.asarray(positions,dtype=float);times=np.asarray(times,dtype=float)
        if path.ndim!=2 or path.shape[1]!=7 or len(path)<2 or times.shape!=(len(path),):
            raise ValueError('Invalid time-parameterized trajectory')
        if not np.isfinite(path).all() or not np.isfinite(times).all() or times[0]!=0 or not (np.diff(times)>0).all():
            raise ValueError('Trajectory has invalid values/timing')
        if (path<self.lower).any() or (path>self.upper).any():
            raise ValueError('Trajectory crosses qualified joint limits')
        if self.faulted:
            raise RuntimeError('RealMan adapter is fault-latched')
        try:
            self.stop_token.check()
            if abs(self.read_joints()-path[0]).max()>self.start_gap:
                raise RuntimeError('stale_robot_start_state')
            start=time.monotonic();last_check=start
            for q,t in zip(path,times):
                self.stop_token.check()
                due=start+float(t)
                self.stop_token.wait(max(0.,due-time.monotonic()))
                if time.monotonic()-due>self.max_jitter:
                    raise TimeoutError('trajectory_stream_deadline_missed')
                with self._lock:
                    self._ok(self.sdk.rm_movej_canfd(np.rad2deg(q).tolist(),False),'movej_canfd')
                if time.monotonic()-last_check>=.1:
                    if abs(self.read_joints()-q).max()>self.following_error:
                        raise RuntimeError('joint_following_error')
                    last_check=time.monotonic()
            deadline=time.monotonic()+3.
            while True:
                self.stop_token.check()
                if abs(self.read_joints()-path[-1]).max()<=self.endpoint_error:
                    break
                if time.monotonic()>deadline:
                    raise TimeoutError('joint_endpoint_not_reached')
                self.stop_token.wait(.04)
            self.events.emit('trajectory_finished',stage=stage,points=len(path),duration_s=float(times[-1]))
        except BaseException:
            try:
                self.controlled_stop()
            except Exception as stop_error:
                self.events.emit('stop_failed',error=str(stop_error),requires_physical_estop=True)
            raise

    def close(self):
        if self.connected:
            with self._lock:
                self.sdk.rm_delete_robot_arm()
                self.connected=False


def time_parameterize(path,fk_xyz,*,speed_mps,hz=30,joint_speed_rad_s=.3,joint_accel_rad_s2=.6):
    """Conservative stop-at-waypoint quintic interpolation; never skips a waypoint.

    The endpoint FK arc is rechecked on emitted samples and the whole schedule
    slowed if necessary. This bounds the sampled reference speed, not hardware
    tracking error; hardware following/endpoint checks remain mandatory.
    """
    path=np.asarray(path,dtype=float)
    if path.ndim!=2 or path.shape[1]!=7 or len(path)<2 or not np.isfinite(path).all():
        raise ValueError('Expected finite Nx7 path with at least two points')
    speed=finite(speed_mps,'speed_mps',.001,.05)
    hz=finite(hz,'hz',10,100)
    vmax=finite(joint_speed_rad_s,'joint_speed_rad_s',.01,.5)
    amax=finite(joint_accel_rad_s2,'joint_accel_rad_s2',.01,1.)
    xyz=np.asarray([fk_xyz(q) for q in path],dtype=float)
    output=[path[0]];times=[0.]
    for index,(a,b) in enumerate(zip(path,path[1:])):
        delta=abs(b-a).max()
        if delta<1e-10:
            continue
        duration=max(1.875*delta/vmax,np.sqrt(5.8*delta/amax),
                     2*np.linalg.norm(xyz[index+1]-xyz[index])/speed,2/hz)
        count=max(2,int(np.ceil(duration*hz)))
        duration=count/hz
        for j in range(1,count+1):
            u=j/count;scale=10*u**3-15*u**4+6*u**5
            output.append(a+(b-a)*scale)
            times.append(times[-1]+1/hz)
    if len(output)==1:
        output.append(path[0].copy());times.append(1/hz)
    out=np.asarray(output);ts=np.asarray(times)
    xyz=np.asarray([fk_xyz(q) for q in out],dtype=float)
    cart=np.linalg.norm(np.diff(xyz,axis=0),axis=1)/np.diff(ts)
    vel=np.diff(out,axis=0)/np.diff(ts)[:,None]
    acc=np.diff(vel,axis=0)/(np.diff(ts)[:-1,None]) if len(vel)>1 else np.zeros((1,7))
    slowdown=max(1.,float(cart.max(initial=0))/speed,float(abs(vel).max(initial=0))/vmax,
                 float(np.sqrt(abs(acc).max(initial=0)/amax)))
    if not np.isfinite(slowdown):
        raise ValueError('Invalid FK/timing result')
    return out,ts*slowdown
