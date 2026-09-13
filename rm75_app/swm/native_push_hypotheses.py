"""Original native PushT hypothesis factory, without any execution capability."""
from contextlib import contextmanager
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import numpy as np
from .scene import SceneInvalid,digest
from .identification import PhysicsParameters
from .skills import PlannedSkill
from .native_skills import NativePrimitive,NativeStage,NativeStageState
from rm75_app.workcell.io import atomic_json


class NativePushHypothesisFactory:
    """One bounded planning transaction; close deletes its private artifacts.

    Candidate proposals retain the original response model. Material parameters
    affect actual future PhysX evaluation, never surrogate friction_scales.
    GPU work and physics subprocesses are serialized even if callers use threads.
    The caller must still install a current-scene atomic auditor and executor.
    """
    def __init__(self,profile,geometry,urdf,*,python,directory,check=lambda:None,max_candidates=2):
        if type(max_candidates) is not int or not 1<=max_candidates<=4:
            raise ValueError('Native candidate budget must be 1..4')
        if not Path(python).is_file() or not Path(urdf).is_file():
            raise FileNotFoundError('Existing native interpreter and original URDF required')
        self.profile=copy.deepcopy(profile);self.geometry=copy.deepcopy(geometry)
        self.urdf=str(urdf);self.python=str(python);self.check=check;self.count=max_candidates
        self.lock=threading.RLock();self.closed=False;self.key=None;self.candidates=[];self.predictions={}
        self.temporary=tempfile.TemporaryDirectory(prefix='native_push_',dir=directory)
        self.directory=Path(self.temporary.name)

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def close(self):
        with self.lock:
            self.closed=True;self.candidates.clear();self.predictions.clear();self.temporary.cleanup()
    def __call__(self):
        if self.closed:raise RuntimeError('Native planning transaction closed')
        return _NativePushContext(self)
    def _check(self):
        if self.closed:raise RuntimeError('Native planning transaction closed')
        self.check()
    @contextmanager
    def operation(self):
        """A failed or cancelled operation invalidates the entire transaction."""
        with self.lock:
            try:
                self._check()
                yield
            except BaseException:
                self.close()
                raise
    @staticmethod
    def _read(path):
        if path.stat().st_size>16000000:raise SceneInvalid('Native planning artifact exceeds budget')
        return json.loads(path.read_bytes())

    @contextmanager
    def _executor(self,snapshot):
        from rm75_app.planning.backends.curobo2 import Curobo2Backend,Curobo2BackendConfig
        from rm75_app.pusht.motion import CuroboPushExecutor
        from rm75_app.pusht.model import Config
        from tools.pusht_physics_planner import audit_predicted_retreat
        class Audited(CuroboPushExecutor):
            def plan_push(self,push,observation,**kwargs):
                prepared=super().plan_push(push,observation,**kwargs)
                audit_predicted_retreat(self,prepared,push,observation)
                return prepared
        with Curobo2Backend(Curobo2BackendConfig(**self.profile['planner'])) as backend:
            arm=SimpleNamespace(hz=30.,read_joints=lambda:np.asarray(snapshot['robot']['positions'],float).copy())
            executor=Audited(backend,arm,Config.from_dict(self.profile['model']),self.profile['motion'],
                SimpleNamespace(check=self._check),SimpleNamespace(emit=lambda *a,**k:None),None)
            executor.names=tuple(snapshot['robot']['joint_names'])
            yield executor

    def _compile(self,index,candidate,snapshot):
        root=Path(__file__).resolve().parents[2];folder=self.directory/f'candidate_{index}';folder.mkdir()
        atomic_json(folder/'plan.json',candidate);atomic_json(folder/'scene.json',dict(initial_snapshot=snapshot))
        atomic_json(folder/'geometry.json',self.geometry)
        command=[sys.executable,str(root/'tools/run_network_isolated.py'),'--',self.python,
            str(root/'tools/compile_swm_future_push.py'),'--plan',str(folder/'plan.json'),
            '--transition',str(folder/'scene.json'),'--geometry',str(folder/'geometry.json'),
            '--urdf',self.urdf,'--output',str(folder/'fk')]
        logfile=folder/'compile.log';deadline=time.monotonic()+180
        with logfile.open('wb') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while process.poll() is None:
                    self._check()
                    if time.monotonic()>deadline or logfile.stat().st_size>16000000:
                        raise TimeoutError('Native FK compilation exceeded resource budget')
                    time.sleep(.05)
                if process.returncode:raise RuntimeError('Original native FK compiler rejected candidate')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
        return self._read(folder/'fk/future_tool_motion.json'),folder

    def _prepare(self,request,snapshot):
        self._check()
        canonical=dict(snapshot);sid=canonical.pop('snapshot_id')
        if (digest(canonical)!=sid or not snapshot['valid'] or request.skill!='push' or
                request.target_reference_id is not None or snapshot['robot']['holding']!='empty' or
                [oid for oid,obj in snapshot['objects'].items() if not obj['fixed']]!=[request.object_id]):
            raise SceneInvalid('Native PushT factory requires an exact resolved idle empty-hand scene')
        key=digest(dict(request=request.as_dict(),snapshot=sid))
        if self.key is not None:
            if key!=self.key:raise SceneInvalid('Use a new native factory for a changed planning transaction')
            return
        self.key=key
        goal=np.asarray(request.target,float)
        self.goal=[float(goal[0,3]),float(goal[1,3]),float(np.arctan2(goal[1,0],goal[0,0]))]
        self.request=request;self.snapshot=copy.deepcopy(snapshot)
        from .future_candidates import generate_original_push_candidates
        from rm75_app.planning.contracts import JointTrajectory
        with self._executor(snapshot) as executor:
            candidates=generate_original_push_candidates(executor,snapshot,self.goal,count=self.count)
        state=NativeStageState(None,'empty',gripper_positions=snapshot['robot']['gripper_joint_positions'])
        for index,candidate in enumerate(candidates):
            motion,folder=self._compile(index,candidate,snapshot)
            stages=tuple(NativeStage(row['stage'],JointTrajectory(tuple(snapshot['robot']['joint_names']),
                np.asarray(row['positions'],float),dt=np.diff(row['times'])),state_before=state,state_after=state,
                contact_objects=(request.object_id,) if row['stage'] in ('contact','push','retreat') else (),
                allow_start_contact_escape=row['stage']=='retreat') for row in candidate['stages'])
            primitive=NativePrimitive('push',request.object_id,stages)
            self.candidates.append(dict(primitive=primitive,motion=motion,folder=folder))

    def _predict(self,candidate,parameters):
        self._check();PhysicsParameters(**parameters)
        from .physics_replay import SubprocessReplayWorld
        from .future_collision_samples import future_collision_samples
        parameter_id=digest(parameters);key=(candidate['primitive'].fingerprint(),parameter_id)
        if key in self.predictions:
            path,expected=self.predictions[key];result=self._read(path)
            if digest(result)!=expected:raise SceneInvalid('Private prediction artifact changed')
            return result
        if len(self.predictions)>=32*self.count:raise SceneInvalid('Native hypothesis budget exhausted')
        motion=candidate['motion'];rows=motion['samples']
        action=dict(source='planned_trajectory',time_s=[row['time_s'] for row in rows],
            T_world_tcp=[row['T_world_tcp'] for row in rows],stages=[row['stage'] for row in rows])
        request=dict(mode='future_prediction',hypothesis_id=parameter_id,parameters=parameters,
            initial_snapshot=self.snapshot,object_id=self.request.object_id,planned_action=action,
            plan_action_digest=digest(action),future_tool_motion=motion,sample_times=action['time_s'])
        world=SubprocessReplayWorld(request,python=self.python,native_tool_geometry=self.geometry,
            directory=candidate['folder'],check=self._check)
        try:result=world.replay()
        finally:world.close()
        future_collision_samples(motion,result,self.snapshot,self.request.object_id)
        if result.get('parameters')!=parameters:raise SceneInvalid('Native prediction parameter mismatch')
        path=candidate['folder']/parameter_id/'result.json'
        self.predictions[key]=(path,digest(result))
        return result


class _NativePushContext:
    owns_real_executor=False
    def __init__(self,factory):self.factory=factory;self.closed=False
    def close(self):self.closed=True
    def solve_candidates(self,request,snapshot,parameters):
        if self.closed:raise RuntimeError('Native context closed')
        factory=self.factory
        with factory.operation():
            factory._prepare(request,snapshot);plans=[]
            for candidate in factory.candidates:
                result=factory._predict(candidate,parameters);primitive=candidate['primitive']
                plans.append(PlannedSkill(digest(request.as_dict()),snapshot['snapshot_id'],primitive.fingerprint(),
                    primitive,result['T_world_object'][-1],'original_native_PushT_hypothesis_factory'))
            return plans
    def evaluate(self,plan,snapshot,parameters):
        if self.closed:raise RuntimeError('Native context closed')
        factory=self.factory
        with factory.operation():
            factory._prepare(factory.request,snapshot)
            if (plan.skill_digest!=digest(factory.request.as_dict()) or
                    plan.source_snapshot_id!=snapshot['snapshot_id'] or
                    not isinstance(plan.payload,NativePrimitive) or plan.payload.fingerprint()!=plan.payload_digest):
                raise SceneInvalid('Native hypothesis plan binding changed')
            matches=[row for row in factory.candidates if row['primitive'].fingerprint()==plan.payload_digest]
            if len(matches)!=1:raise SceneInvalid('Unknown original native candidate')
            candidate=matches[0];result=factory._predict(candidate,parameters)
            from .future_push_audit import audit_future_push
            from .future_push_score import score_audited_future_push
            from .candidate_rejection import CandidateInfeasible
            try:
                with factory._executor(snapshot) as executor:
                    audit=audit_future_push(executor,candidate['motion'],result,snapshot,factory.request.object_id)
                    score=score_audited_future_push(candidate['motion'],result,audit,snapshot,
                        factory.request.object_id,factory.goal,executor.config)
            except CandidateInfeasible as exc:
                # The native audit restores its scene and the owned GPU context
                # exits BEFORE returning ordinary infeasibility. All other errors
                # still reach operation(), which closes the entire transaction.
                return dict(snapshot_id=snapshot['snapshot_id'],payload_digest=plan.payload_digest,
                    feasible=False,cost=None,parameters=copy.deepcopy(parameters),
                    rejection=dict(kind='native_candidate_infeasible',stage=exc.stage,sample=exc.sample,
                        reason=str(exc)[:4096],parameters_digest=digest(parameters)),
                    private_native_context_exited=True,full_primitive_audit_issued=False,execution_authorized=False)
            return dict(snapshot_id=snapshot['snapshot_id'],payload_digest=plan.payload_digest,feasible=True,
                cost=score['cost'],expected_object_pose=result['T_world_object'][-1],parameters=copy.deepcopy(parameters),
                audit=audit,score=score,full_primitive_audit_issued=False,execution_authorized=False)
