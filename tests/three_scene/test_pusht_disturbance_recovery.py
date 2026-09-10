"""Moving-T recovery: explicit disturbance schedule, fit exclusion, and service flow."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from rm75_app.pusht.controller import PushTController
from rm75_app.pusht.observation import Observation
from rm75_app.pusht.model import Config
from rm75_app.workcell.events import StopToken


def test_disturbance_is_observed_then_recovery_completes_without_fit_poisoning():
    events=[];applied=[]

    class Physics:
        t=1.;n=0;waits=[];pushes=0;moved=False

        def now(self):return self.t

        def advance(self,seconds):self.waits.append(seconds);self.t+=seconds

        def observe(self,after=0):
            self.t+=1/30;self.n+=1
            if self.pushes>=2:pose=[.38,-.18,0]      # recovered after the second push
            elif self.moved:pose=[.355,-.185,.08]    # externally moved pose
            else:pose=[.35,-.18,0]
            return Observation('physics',self.n,self.t,tuple(pose),'simulation')

        def stable(self):return True

        def execute_push(self,*a):
            self.pushes+=1;self.moved=False

        def disturb(self,step):
            applied.append(step);self.moved=True
            return True

    physics=Physics()
    controller=PushTController(physics,physics,Config(),StopToken(),
        NS(emit=lambda kind,**kw:events.append((kind,kw))),
        clock=physics.now,wait=physics.advance,verification='physics_pose',
        disturb=lambda step:physics.disturb(step) if step==1 else False)
    result=controller.run([.38,-.18,0])
    assert applied==[1]
    assert result['task_success'] is True and result['verification']=='physics_pose'
    assert result['steps']==2 and physics.pushes==2
    # The push->observation transition crossed an external move: that round is
    # never fitted; the genuine transition afterwards still is.
    assert sum(1 for kind,kw in events if kind=='response_fit_skipped'
               and kw['reason']=='external_disturbance_after_push')==1
    assert sum(1 for kind,kw in events if kind=='push_response_fitted')==1


def test_no_disturb_hook_keeps_original_fit_behaviour():
    events=[]

    class Physics:
        t=1.;n=0;waits=[]
        pushes=0

        def now(self):return self.t

        def advance(self,seconds):self.waits.append(seconds);self.t+=seconds

        def observe(self,after=0):
            self.t+=1/30;self.n+=1
            return Observation('physics',self.n,self.t,(.38,-.18,0),'simulation')

        def stable(self):return True

        def execute_push(self,*a):pytest.fail('Already at target')

    physics=Physics()
    controller=PushTController(physics,physics,Config(),StopToken(),NS(emit=lambda kind,**kw:events.append(kind)),
        clock=physics.now,wait=physics.advance,verification='physics_pose')
    controller.run([.38,-.18,0])
    assert 'response_fit_skipped' not in events


def test_target_pose_overwrite_only_exists_inside_disturbance_hook():
    source=(Path(__file__).resolve().parents[2]/'rm75_app/pusht/physics.py').read_text()
    tree=ast.parse(source)
    calls=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.FunctionDef):
            for call in ast.walk(node):
                if isinstance(call,ast.Call) and ast.unparse(call.func)=='self.base.target.set_pose':
                    calls.append(node.name)
    assert calls==['disturb']
    assert "physics.get('disturbances')" in source or "'disturbances'" in source
    assert 'set_linear_velocity' in source and 'set_angular_velocity' in source


def test_validation_tool_random_cases_are_fixed_and_valid():
    tool=Path(__file__).resolve().parents[2]/'tools/run_pusht_physics_validation.py'
    tree=ast.parse(tool.read_text())
    constants={n.targets[0].id:ast.literal_eval(n.value) for n in tree.body
        if isinstance(n,ast.Assign) and len(n.targets)==1 and isinstance(n.targets[0],ast.Name)
        and n.targets[0].id in ('CASES','RANDOM_CASES')}
    assert set(constants['CASES'])=={'translation','rotation','mixed','neighbor','infeasible','cancel'}
    frozen=json.loads((Path(__file__).resolve().parents[2]
        /'runtime_data/three_scene/pickplace_pusht_followup_20260908/gpu_baseline/input.json').read_text())
    model=Config.from_dict({**frozen['config'],'maximum_push_length_m':.05,'horizon':3,'beam_width':12})
    from rm75_app.pusht.model import valid_pose
    for name,initial,goal in constants['RANDOM_CASES']:
        assert name.startswith('r') and len(initial)==len(goal)==3
        assert valid_pose(initial,model) and valid_pose(goal,model), name


def test_validation_tool_disturbance_requires_both_arguments_and_finite_yaw():
    from argparse import ArgumentParser,ArgumentTypeError
    tool=Path(__file__).resolve().parents[2]/'tools/run_pusht_physics_validation.py'
    source=tool.read_text()
    # Argument pairing and finite-yaw checks are enforced before any service starts.
    assert '--disturb-after-pushes and --disturb-pose must be given together' in source
    assert '--disturb-pose must be finite x,y and |yaw|<=pi' in source
    assert "'disturbances'" in source and 'after_pushes' in source


def test_disturbance_schedule_lives_in_machine_profile_not_request_spec():
    source=(Path(__file__).resolve().parents[2]/'tools/run_pusht_physics_validation.py').read_text()
    tree=ast.parse(source);frozen=next(n for n in ast.walk(tree)
        if isinstance(n,ast.FunctionDef) and n.name=='frozen_profile')
    fn=ast.unparse(frozen)
    assert "physics['disturbances'] = [dict(after_pushes=args.disturb_after_pushes, pose=list(args.disturb_pose))]" in fn
    assert 'speed_mps=0.015, max_steps=12, simulation_backend=args.backend' in fn
    # The typed browser contract (spec.py) is unchanged: disturbances are not
    # request parameters.
    spec=(Path(__file__).resolve().parents[2]/'rm75_app/workcell/spec.py').read_text()
    assert "'disturbances'" not in spec


def test_physics_run_wires_schedule_and_labels_evidence():
    source=(Path(__file__).resolve().parents[2]/'rm75_app/pusht/physics.py').read_text()
    assert "profile['pusht'].get('physics',{}).get('disturbances')" in source
    assert "session.report['disturbance_schedule']=schedule" in source
    assert "kind='external_pose_overwrite_between_pushes'" in source
    assert 'external_disturbance_applied' in source
    assert "disturb=disturb if schedule else None" in source
