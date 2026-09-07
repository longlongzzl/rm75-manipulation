#!/usr/bin/env python3
"""Real cuRobo2 GPU / synthetic scene validation. NEVER a hardware qualification."""
import argparse
from copy import copy
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import time
import traceback
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.planning.backends.curobo2 import Curobo2Backend, Curobo2BackendConfig
from rm75_app.pusht.model import Config, choose_push
from rm75_app.pusht.observation import Observation
from rm75_app.pusht.motion import CuroboPushExecutor
from rm75_app.pusht.cartesian_ik import PushPathRejected
from rm75_app.planning.contracts import CollisionObject,Pose,PlanningScene,JointConfiguration
from rm75_app.workcell.events import StopToken
from rm75_app.workcell.io import atomic_json


class RecordedBackend(Curobo2Backend):
    def __init__(self, config):
        super().__init__(config)
        self.records = []

    def plan_candidates(self, request):
        self.current_stage = request.candidates[0].candidate_id
        result = super().plan_candidates(request)
        self.records.append({'kind': 'plan', 'stage': request.candidates[0].candidate_id,
                             'plans': [dict(success=p.success, status=p.status,
                                            diagnostics=p.diagnostics) for p in result.plans]})
        return result

    def plan_linear_candidates(self, request, **kwargs):
        self.current_stage = request.candidates[0].candidate_id
        result = super().plan_linear_candidates(request, **kwargs)
        self.records.append({'kind': 'plan', 'primitive': 'linear', 'options': kwargs,
            'stage': self.current_stage, 'plans': [dict(success=p.success,status=p.status,
            diagnostics=p.diagnostics) for p in result.plans]})
        return result

    def _ensure_planner(self):
        planner = super()._ensure_planner()
        if not getattr(planner, '_pusht_record_ik', False):
            planner._pusht_record_ik = True
            original = planner.ik_solver.solve_pose
            def solve(*args, **kwargs):
                result = original(*args, **kwargs)
                row = dict(kind='native_ik', stage=getattr(self,'current_stage',None),
                           success=result.success.detach().cpu().tolist())
                for key in ('position_error','rotation_error'):
                    value = getattr(result,key,None)
                    if value is not None:row[key]=value.detach().cpu().tolist()
                self.records.append(row)
                try:
                    mods = self._import_modules()
                    state = mods['JointState'].from_position(result.solution.reshape(-1,7),
                        joint_names=list(planner.joint_names))
                    row['contacts'] = self._collision_diagnostics_for_states(planner,{'ik':state})
                except Exception as exc:
                    row['diagnostic_error'] = str(exc)
                return result
            planner.ik_solver.solve_pose = solve
        return planner

    def solve_pose_ik_variants(self, request):
        self.current_stage=request.candidates[0].candidate_id
        result=super().solve_pose_ik_variants(request)
        self.records.append(dict(kind='cartesian_ik',stage=self.current_stage,
                                 successful_variants=len(result)))
        return result

    def _collision_diagnostics_for_states(self, planner, states, **kwargs):
        result = super()._collision_diagnostics_for_states(planner, states, **kwargs)
        self.records.append({'kind': 'collision_audit',
                             'samples': sum(int(s.position.shape[0]) for s in states.values() if s is not None),
                             'contacts': result})
        return result


EXECUTION_GATE_CASES = (
    'stale', 'replayed', 'non_monotonic', 'changed_session', 'low_confidence',
    'wrong_frame', 'simulation_source', 'translation_drift', 'yaw_drift', 'joint_drift',
)


def audit_execution_gates(executor, prepared, push, observation):
    """Injected-input checks AFTER a real GPU plan, never hardware/camera evidence.

    Reuse only this exact prepared chain. A copied executor exercises the real
    execute_push preflight, with an arm sink that refuses EVERY execute call.
    No tracker timestamp, backend scene, or production profile is changed.
    """
    now = time.time()
    previous = replace(observation, captured_at=now-executor.config.max_observation_age_s-2)
    fresh = replace(previous, sequence=previous.sequence+1, captured_at=now,
                    source='live_tracker')  # Explicit test injection, NOT a live observation.
    x, y, yaw = fresh.pose
    cases = {
        'stale': (replace(fresh, captured_at=now-executor.config.max_observation_age_s-1),
                  ValueError, 'stale_or_future_observation'),
        'replayed': (replace(fresh, sequence=previous.sequence),
                     ValueError, 'replayed_or_changed_capture_session'),
        'non_monotonic': (replace(fresh, captured_at=previous.captured_at),
                          ValueError, 'stale_or_future_observation'),
        'changed_session': (replace(fresh, session_id=previous.session_id+'_restarted'),
                            ValueError, 'replayed_or_changed_capture_session'),
        'low_confidence': (replace(fresh, confidence=.1), ValueError, 'low_confidence_observation'),
        'wrong_frame': (replace(fresh, frame='camera'), ValueError, 'base_link'),
        'simulation_source': (replace(fresh, source='simulation'), ValueError, 'live observation source'),
        'translation_drift': (replace(fresh, pose=(x+.004, y, yaw)), RuntimeError, 'Object moved while planning'),
        'yaw_drift': (replace(fresh, pose=(x, y, yaw+.05)), RuntimeError, 'Object moved while planning'),
        'joint_drift': (fresh, RuntimeError, 'Robot moved while planning'),
    }
    rows = []
    for name in EXECUTION_GATE_CASES:
        current, error_class, message = cases[name]
        calls = {'plan': 0, 'observe': 0, 'execute': 0}
        test_executor = copy(executor)

        def prepared_only(*args):
            calls['plan'] += 1
            return prepared

        def observe(*, after):
            calls['observe'] += 1
            if after != previous.captured_at:
                raise AssertionError('Execution did not request a newer observation')
            return current

        class NoMotionGateArm:
            start_gap = .05

            def read_joints(self):
                joints = prepared.start_q.copy()
                if name == 'joint_drift':
                    joints[0] += self.start_gap+.001
                return joints

            def execute(self, *args, **kwargs):
                calls['execute'] += 1
                raise AssertionError('No-motion gate audit cannot execute')

        test_executor.plan_push = prepared_only
        test_executor.observer = SimpleNamespace(observe=observe)
        test_executor.arm = NoMotionGateArm()
        row = dict(case=name, rejected=False, injected_observation=True,
                   actual_camera_observation=False, reused_gpu_prepared_chain=True)
        try:
            test_executor.execute_push(push, previous)
        except Exception as exc:
            row.update(error_type=type(exc).__name__, error=str(exc),
                rejected=isinstance(exc, error_class) and message in str(exc))
        row.update(calls)
        row['passed'] = bool(row['rejected'] and calls == {'plan': 1, 'observe': 1, 'execute': 0})
        rows.append(row)
    return rows


def validation_passed(report, *, audit_blocker=False, audit_gates=False):
    """Planning alone is insufficient if reporting or the dynamics audit failed."""
    if not report.get('complete_chain') or report.get('error'):
        return False
    stages=report.get('stages',[])
    if [row.get('stage') for row in stages]!=['approach','descend','contact','push','retreat']:
        return False
    limits={'max_tcp_speed_mps':report['config']['speed_mps'],
            'max_joint_speed_rad_s':report['motion']['joint_speed_rad_s'],
            'max_joint_accel_rad_s2':report['motion']['joint_accel_rad_s2']}
    for row in stages:
        for key,limit in limits.items():
            value=row.get(key,float('nan'))
            if not np.isfinite(value) or value<0 or value>limit+1e-9:
                return False
    if audit_blocker and report.get('unrelated_obstacle_audit',{}).get('rejected') is not True:
        return False
    if audit_gates:
        rows = report.get('execution_gate_audits', [])
        if [row.get('case') for row in rows] != list(EXECUTION_GATE_CASES):
            return False
        if any(row.get('passed') is not True or row.get('execute') != 0 for row in rows):
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=['baseline', 'orthogonal_tool', 'rotated', 'neighbor_blocked',
                        'rotated_orthogonal','orthogonal_neighbor_blocked','yaw_matched_tool'], default='baseline')
    parser.add_argument('--fixture', choices=['raised_table', 'low_table'], default='raised_table')
    parser.add_argument('--speed',type=float,default=.015)
    parser.add_argument('--audit-blocker',action='store_true')
    parser.add_argument('--audit-execution-gates',action='store_true',
                        help='Inject stale/replayed/drift inputs after GPU planning; no execute allowed')
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    config = Config.from_dict({'speed_mps':args.speed})
    pose = (.35, -.18, .3 if args.case in ('rotated','rotated_orthogonal','yaw_matched_tool') else 0.)
    goal = (.38, -.18, pose[2])
    # Authored simulation fixture, not measurements of the user's workcell.
    motion = dict(tool_frame='gripper_tcp', push_tcp_z_m=.15, hover_clearance_m=.08,
        object_centroid_z_m=.14, object_height_m=.04,
        tool_quaternion_wxyz=[0., 2**-.5, 2**-.5, 0.],
        pusher_contact_links=['left_pad', 'right_pad', 'gripper_Left_1_Link',
            'gripper_Left_2_Link', 'gripper_Left_Support_Link', 'gripper_Right_1_Link',
            'gripper_Right_2_Link', 'gripper_Right_Support_Link'],
        tool_collision_geometry_verified=True,  # ONLY the unchanged simulated gripper spheres.
        corridor_tolerance_m=.003, orientation_tolerance_rad=.05,
        joint_speed_rad_s=.25, joint_accel_rad_s2=.5,
        static_collision_objects=[dict(name='simulation_table', kind='cuboid',
            position=[.40, -.18, .10], quaternion_wxyz=[1,0,0,0], dimensions=[.40,.28,.04])])
    if args.case in ('orthogonal_tool','rotated_orthogonal','orthogonal_neighbor_blocked'):
        motion['tool_quaternion_wxyz'] = [0., 1., 0., 0.]
    if args.case=='yaw_matched_tool':
        # Separate authored fixture: rotate tool yaw with the target, preserving
        # their relative geometry. Do not overwrite the fixed-tool failure.
        motion['tool_quaternion_wxyz']=[0.,float(np.cos(.15)),float(np.sin(.15)),0.]
    if args.fixture == 'low_table':
        # Separate authored fixture selected after a reachability diagnostic;
        # retain the original raised-table failures in the experiment record.
        motion.update(push_tcp_z_m=.02, object_centroid_z_m=.01)
        motion['static_collision_objects'][0]['position'][2] = -.03
    push, prediction = choose_push(pose, goal, config)
    if args.case in ('neighbor_blocked','orthogonal_neighbor_blocked'):
        entry = np.asarray(push.contact) - np.asarray(push.direction) * config.approach_gap_m
        motion['static_collision_objects'].append(dict(name='blocking_neighbor',kind='cuboid',
            position=[*entry,motion['push_tcp_z_m']+motion['hover_clearance_m']],
            quaternion_wxyz=[1,0,0,0],dimensions=[.08,.08,.08]))
    planner_config = Curobo2BackendConfig()
    report = dict(case=args.case, fixture=args.fixture, scenario='authored_simulation_fixture',
        gpu_backend='curobo2', hardware_connected=False, execute_real=False,
        hardware_profile_qualified=False, verified_physical_success=None,
        config=asdict(config), motion=motion, push=push.as_dict(), prediction=prediction,
        planner_config=json.loads(json.dumps(asdict(planner_config),default=str)),
        events=[], complete_chain=False)
    atomic_json(output/'input.json', report)
    backend = RecordedBackend(planner_config)
    started = time.monotonic()
    try:
        native = backend._ensure_planner()
        q = native.default_joint_state.position.reshape(-1)[:7].detach().cpu().numpy()
        backend.set_gripper_collision_state(closed=True)
        class NoMotionArm:
            hz = 30
            def read_joints(self): return q.copy()
            def execute(self, *args, **kwargs): raise AssertionError('GPU validation cannot execute')
        executor = CuroboPushExecutor(backend, NoMotionArm(), config, motion,
            StopToken(output/'STOP'), SimpleNamespace(emit=lambda event,**data:
                report['events'].append(dict(event=event,**data))), None)
        observation = Observation('synthetic_gpu_fixture',1,time.time(),pose,'simulation')
        prepared = executor.plan_push(push,observation)
        report.update(complete_chain=True, stages=[dict(stage=name,positions=path.tolist(),
            times=times.tolist()) for name,path,times in prepared.stages])
        for item,(_,path,times) in zip(report['stages'],prepared.stages):
            xyz=np.array([backend.tool_pose_for_configuration(JointConfiguration(executor.names,q),'gripper_tcp').position for q in path])
            dt=np.diff(times);vel=np.diff(path,axis=0)/dt[:,None]
            item['max_tcp_speed_mps']=float(np.max(np.linalg.norm(np.diff(xyz,axis=0),axis=1)/dt))
            item['max_joint_speed_rad_s']=float(np.max(np.abs(vel)))
            item['max_joint_accel_rad_s2']=float(np.max(np.abs(np.diff(vel,axis=0)/((dt[1:]+dt[:-1])/2)[:,None]))) if len(vel)>1 else 0.
        if args.audit_execution_gates:
            report['execution_gate_audits'] = audit_execution_gates(executor, prepared, push, observation)
        if args.audit_blocker:
            # Inject one unrelated object into an already validated positive
            # path. This exercises the actual GPU collision-audit gate, not IK.
            scene=executor._scene(observation)
            path=next(path for name,path,_ in prepared.stages if name=='push')
            tcp=backend.tool_pose_for_configuration(JointConfiguration(executor.names,path[len(path)//2]),'gripper_tcp')
            blocker=CollisionObject('gpu_audit_neighbor','cuboid',Pose(tcp.position,[1,0,0,0]),dimensions=[.06,.06,.06])
            backend.update_scene(PlanningScene((*scene.objects,blocker),revision='negative_audit'))
            try:
                executor._audit(path,contact=True)
                report['unrelated_obstacle_audit']=dict(rejected=False)
            except PushPathRejected as exc:
                report['unrelated_obstacle_audit']=dict(rejected=True,error=str(exc))
            finally:
                backend.update_scene(scene)
    except Exception as exc:
        report.update(error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
    finally:
        report.update(elapsed_s=time.monotonic()-started, backend_records=backend.records)
        backend.__exit__(None,None,None)
        report['validation_success']=validation_passed(report,audit_blocker=args.audit_blocker,
                                                       audit_gates=args.audit_execution_gates)
        atomic_json(output/'result.json',report)
    print(json.dumps({'case':args.case,'complete_chain':report['complete_chain'],
                      'validation_success':report['validation_success'],
                      'error':report.get('error'),'elapsed_s':report['elapsed_s']}))
    return 0 if report['validation_success'] else 42


if __name__ == '__main__':
    raise SystemExit(main())
