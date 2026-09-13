import copy
from types import SimpleNamespace
import numpy as np
import pytest
from .conftest import pose
from rm75_app.swm.scene import digest,SceneInvalid
from rm75_app.swm.identification import (PhysicsParameters,ParameterBounds,sample_hypotheses,
    validate_transition,IsolatedReplayPool,infer_posterior,select_robust_candidate)
from rm75_app.swm.program import compile_skill_code,run_program,ProgramRejected,SWMAgentPlanner
from rm75_app.swm.skills import SkillRequest
from rm75_app.swm.physics_replay import MeasuredTCPProgram,interpolate_pose


def transition(rig,dense=True):
    snap=rig.sync.sync('before_push');t=[0.,.5,1.] if dense else [0.,1.]
    initial=snap['objects']['a']['measured']['T_world_object']
    return dict(schema='rm75_measured_transition_v1',domain='fixture',observation_source='fixture',object_id='a',support_id='table',
        calibration_id=snap['calibration_id'],sensor_session=snap['sensor_session'],mesh_sha256=snap['assets']['asset']['mesh_sha256'],
        initial_snapshot=snap,initially_settled=True,settling_evidence='fixture',action_id='physical-action-id',intervened=False,
        holding_changed=False,actual_action=dict(source='measured_feedback',time_s=[0.,.5,1.],
            T_world_tcp=[snap['robot']['T_world_tcp'],pose(.35,z=.2),pose(.4,z=.2)],stages=['push']*3),
        object_time_s=t,T_world_object=[initial]+[pose(.3+.05*x,z=.03) for x in t[1:]],
        object_accepted=[True]*len(t),object_sequences=list(range(len(t))))


class Replay:
    domain='fixture'
    def __init__(self,req,closed):self.req=req;self.closed=closed
    def replay(self):
        r=self.req
        # Intentionally synthetic numerical response: tests orchestration, NOT physical validity.
        mu=r['parameters']['dynamic_friction'];times=r['sample_times']
        return dict(hypothesis_id=r['hypothesis_id'],action_digest=r['action_digest'],
            transition_digest=r['transition_digest'],parameters=r['parameters'],initial_snapshot_id=r['initial_snapshot']['snapshot_id'],
            time_s=times,T_world_object=[pose(.3+.1*(1-mu)*t,z=.03) for t in times],valid=True)
    def close(self):self.closed.append(self.req['hypothesis_id'])


def hypotheses():
    return [dict(id='h'+str(i),parameters=PhysicsParameters(.8,mu,1000+i*100).as_dict()) for i,mu in enumerate([.2,.5,.7])]


def test_replay_uses_same_measured_action_and_initial_state(rig):
    data=transition(rig);seen=[];closed=[]
    def factory(req):seen.append(copy.deepcopy(req));return Replay(req,closed)
    checked,rows=IsolatedReplayPool(factory,workers=2).run(data,hypotheses())
    result=infer_posterior(data,hypotheses(),rows)
    assert len(closed)==3 and result['best_fit_id']=='h1' and result['updated']
    assert len({r['action_digest'] for r in seen})==1
    assert len({r['initial_snapshot']['snapshot_id'] for r in seen})==1
    assert not result['unique_parameters_identified'] and not result['density_informative']
    rig.world.update_physics('a',result,expected_physics_revision=0)
    assert rig.world.physics_revision==1
    with pytest.raises(SceneInvalid):rig.world.update_physics('a',result,expected_physics_revision=1)


@pytest.mark.parametrize('change',['predicted','intervention','attachment','no_settle','reorder','bad_t0','no_support','unaccepted','model','source','seq'])
def test_reject_invalid_physics_evidence(rig,change):
    value=transition(rig)
    if change=='predicted':value['actual_action']['source']='planned_trajectory'
    if change=='intervention':value['intervened']=True
    if change=='attachment':value['holding_changed']=True
    if change=='no_settle':value['initially_settled']=False
    if change=='reorder':value['object_time_s']=[0,1,.5]
    if change=='bad_t0':value['T_world_object'][0]=pose(.9)
    if change=='no_support':value['support_id']='a'
    if change=='unaccepted':value['object_accepted'][-1]=False
    if change=='model':value['mesh_sha256']='0'*64
    if change=='source':value['observation_source']='predicted_pose'
    if change=='seq':value['object_sequences']=[1,1,2]
    with pytest.raises(ValueError):validate_transition(value)


def test_endpoint_data_label_and_density_stays_broad(rig):
    data=transition(rig,dense=False);_,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,hypotheses())
    posterior=infer_posterior(data,hypotheses(),rows)
    assert posterior['observation_mode']=='endpoint_only' and not posterior['density_informative']
    particles=sample_hypotheses(ParameterBounds(),count=100,seed=42,posterior=posterior)
    assert max(p['parameters']['density_kg_m3'] for p in particles)>1600
    assert min(p['parameters']['density_kg_m3'] for p in particles)<200


def test_uninformative_data_not_a_unique_model(rig):
    data=transition(rig);_,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,hypotheses())
    for row in rows:row['T_world_object']=copy.deepcopy(data['T_world_object'])
    result=infer_posterior(data,hypotheses(),rows)
    assert not result['updated'] and not result['unique_parameters_identified']


def test_failed_models_kept_in_denominator(rig):
    data=transition(rig);h=hypotheses()
    _,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,h)
    rows[1]={'hypothesis_id':'h1','valid':False,'reason':'failure'}
    result=infer_posterior(data,h,rows)
    assert len(result['particles'])==3 and result['particles'][1]['loss'] is None


def test_different_action_or_parameters_cannot_be_mixed(rig):
    data=transition(rig);_,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,hypotheses())
    rows[0]['action_digest']='other-action'
    with pytest.raises(ValueError):infer_posterior(data,hypotheses(),rows)


def test_all_models_wrong_does_not_shrink_to_least_bad(rig):
    data=transition(rig);_,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,hypotheses())
    for index,row in enumerate(rows):
        row['T_world_object']=copy.deepcopy(data['T_world_object'])
        row['T_world_object'][-1][0][3]+=.1+index*.02
    result=infer_posterior(data,hypotheses(),rows)
    assert result['discriminative'] and not result['model_agreement']
    assert not result['updated'] and not result['parameter_update_admissible']
    assert result['rejection_reason']=='absolute_model_mismatch'
    assert [r['weight'] for r in result['particles']]==pytest.approx([1/3]*3)


def test_single_bad_observation_not_hidden_by_average(rig):
    data=transition(rig);_,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,hypotheses())
    for index,row in enumerate(rows):
        row['T_world_object']=copy.deepcopy(data['T_world_object'])
        row['T_world_object'][-1][0][3]+=.010+index*.001
    result=infer_posterior(data,hypotheses(),rows)
    assert result['minimum_loss']<9
    assert not result['model_agreement'] and not result['updated']


def test_model_agreement_is_not_parameter_information(rig):
    data=transition(rig);_,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,hypotheses())
    for row in rows:row['T_world_object']=copy.deepcopy(data['T_world_object'])
    result=infer_posterior(data,hypotheses(),rows)
    assert result['model_agreement'] and not result['discriminative']
    assert result['rejection_reason']=='uninformative' and not result['updated']


def test_real_data_cannot_be_fitted_by_fixture_model(rig):
    data=transition(rig);data['domain']='real';data['observation_source']='foundationpose';snap=data['initial_snapshot'];snap['observation_domain']='real'
    snap.pop('snapshot_id');snap['snapshot_id']=digest(snap)
    _,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,hypotheses())
    assert all(r['valid'] is False for r in rows)


def test_sampler_is_repeatable_and_valid():
    a=sample_hypotheses(ParameterBounds(),count=16,seed=33)
    assert a==sample_hypotheses(ParameterBounds(),count=16,seed=33)
    assert a!=sample_hypotheses(ParameterBounds(),count=16,seed=34)
    assert all(ParameterBounds().contains(PhysicsParameters(**r['parameters'])) for r in a)


@pytest.mark.parametrize('kwargs',[dict(static_friction=.1,dynamic_friction=.2,density_kg_m3=1000),dict(static_friction=.1,dynamic_friction=.05,density_kg_m3=-1)])
def test_bad_physical_values(kwargs):
    with pytest.raises(ValueError):PhysicsParameters(**kwargs)


def test_robust_path_not_just_best_fitting_world():
    posterior={'particles':[{'id':'a','weight':.7},{'id':'b','weight':.3}]}
    candidates=[dict(id='risky',hypothesis_scores={'a':dict(feasible=True,cost=0),'b':dict(feasible=False,cost=0)}),
                dict(id='robust',hypothesis_scores={'a':dict(feasible=True,cost=2),'b':dict(feasible=True,cost=3)})]
    assert select_robust_candidate(candidates,posterior)['id']=='robust'


def test_llm_code_compiles_to_real_atomic_contracts(rig):
    snap=rig.sync.sync('initial')
    result=compile_skill_code('skills.grasp("a", functional_pose="side")\nskills.place("a", target="destination")',
                             world_snapshot=snap,goals={'destination':pose(.4,z=.03)})
    assert result['execution_authorized'] is False
    done=run_program(result,rig.runtime)
    assert done['completed'] and len(done['results'])==2


@pytest.mark.parametrize('code', ['import os','open("/tmp/x")','while True: pass','skills.__class__("a")',
    'skills.grasp("unknown")','skills.place("a", target="target")','skills.grasp("a", **{})',
    'skills.grasp("a", tolerance=999)','skills.grasp("a"); skills.push("b", target="target")',
    'x = skills.grasp("a")','skills.grasp(str("a"))'])
def test_llm_arbitrary_code_or_wrong_dependency_rejected(rig,code):
    snap=rig.sync.sync('initial')
    with pytest.raises(ProgramRejected):compile_skill_code(code,world_snapshot=snap,goals={'target':pose(.4)})
    assert rig.backend.executions==0


def test_failed_atom_stops_program_dependency(rig):
    snap=rig.sync.sync('initial');code='skills.grasp("a")\nskills.place("a", target="target")'
    program=compile_skill_code(code,world_snapshot=snap,goals={'target':pose(.4)})
    rig.backend.on_execute=lambda req:setattr(rig.source,'holding','empty')
    result=run_program(program,rig.runtime)
    assert result['completed'] is False and result['next_skill']==0
    assert not any(x=='before_place' for x in rig.source.boundaries)


def test_agent_calls_only_existing_skills(rig):
    snap=rig.sync.sync('initial');planner=SWMAgentPlanner(lambda msgs:dict(decomposition=['take object'],code='skills.grasp("a")'))
    plan=planner.plan('take it',snap,{},rig.runtime.capabilities())
    assert plan['steps'][0]['skill']=='grasp'
    with pytest.raises(ProgramRejected):planner.plan('take it',snap,{}, {'grasp':False})


def test_tool_trajectory_interpolation_respects_yaw_wrap(rig):
    a=pose(.3,yaw=np.pi-.1);b=pose(.4,yaw=-np.pi+.1)
    middle=interpolate_pose(a,b,.5)
    assert middle[0,3]==pytest.approx(.35) and middle[0,0]<-.99
    program=MeasuredTCPProgram(dict(time_s=[0,1],T_world_tcp=[a,b],stages=['push','retreat']))
    stage,actual=program.sample(.5)
    assert stage=='push' and np.allclose(actual,middle)
