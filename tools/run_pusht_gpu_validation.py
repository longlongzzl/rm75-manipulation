#!/usr/bin/env python3
"""Real cuRobo2 GPU / synthetic scene validation. NEVER a hardware qualification."""
import argparse
from dataclasses import asdict
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

    def _collision_diagnostics_for_states(self, planner, states, **kwargs):
        result = super()._collision_diagnostics_for_states(planner, states, **kwargs)
        self.records.append({'kind': 'collision_audit',
                             'samples': sum(int(s.position.shape[0]) for s in states.values() if s is not None),
                             'contacts': result})
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=['baseline', 'orthogonal_tool', 'rotated', 'neighbor_blocked'], default='baseline')
    parser.add_argument('--fixture', choices=['raised_table', 'low_table'], default='raised_table')
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    config = Config()
    pose = (.35, -.18, .3 if args.case == 'rotated' else 0.)
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
    if args.case == 'orthogonal_tool':
        motion['tool_quaternion_wxyz'] = [0., 1., 0., 0.]
    if args.fixture == 'low_table':
        # Separate authored fixture selected after a reachability diagnostic;
        # retain the original raised-table failures in the experiment record.
        motion.update(push_tcp_z_m=.02, object_centroid_z_m=.01)
        motion['static_collision_objects'][0]['position'][2] = -.03
    push, prediction = choose_push(pose, goal, config)
    if args.case == 'neighbor_blocked':
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
    except Exception as exc:
        report.update(error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
    finally:
        report.update(elapsed_s=time.monotonic()-started, backend_records=backend.records)
        backend.__exit__(None,None,None)
        atomic_json(output/'result.json',report)
    print(json.dumps({'case':args.case,'complete_chain':report['complete_chain'],
                      'error':report.get('error'),'elapsed_s':report['elapsed_s']}))
    return 0 if report['complete_chain'] else 42


if __name__ == '__main__':
    raise SystemExit(main())
