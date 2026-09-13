"""LLM-written skill code -> bounded typed program, WITHOUT exec/eval.

Only a small Python-shaped skill-call language is interpreted. Imports, loops,
attributes other than skills.<atomic_name>, filesystem and arbitrary Python are
not executable. High-level decomposition is separate from native motion code.
"""
from __future__ import annotations
import ast
import copy
import json
from .scene import digest, identifier
from .skills import SkillRequest, SKILLS


class ProgramRejected(ValueError):
    pass


def compile_skill_code(code, *, world_snapshot, goals, maximum_skills=64):
    if not isinstance(code,str) or not 1<=len(code)<=30000:
        raise ProgramRejected('Skill program source is empty or exceeds budget')
    try:tree=ast.parse(code,mode='exec')
    except (SyntaxError,RecursionError) as exc:raise ProgramRejected('Invalid skill-call code') from exc
    if not 1<=len(tree.body)<=maximum_skills:raise ProgramRejected('Program must be bounded nonempty skill calls')
    steps=[]
    for node in tree.body:
        if not isinstance(node,ast.Expr) or not isinstance(node.value,ast.Call):
            raise ProgramRejected('Only direct skills.<name>(...) statements are allowed')
        call=node.value
        if (not isinstance(call.func,ast.Attribute) or not isinstance(call.func.value,ast.Name)
                or call.func.value.id!='skills' or call.func.attr not in SKILLS or len(call.args)!=1):
            raise ProgramRejected('Only registered atomic skill calls with one instance id are allowed')
        if not isinstance(call.args[0],ast.Constant) or not isinstance(call.args[0].value,str):
            raise ProgramRejected('Object identity must be a literal known instance')
        oid=call.args[0].value
        if oid not in world_snapshot['objects']:raise ProgramRejected('Unknown object instance')
        values={}
        for keyword in call.keywords:
            if keyword.arg not in ('target','functional_pose') or keyword.arg in values:
                raise ProgramRejected('Unknown, duplicate or expanded skill argument')
            if not isinstance(keyword.value,ast.Constant) or not isinstance(keyword.value.value,str):
                raise ProgramRejected('Use registered goal/functional-pose ids, not invented coordinates')
            values[keyword.arg]=keyword.value.value
        target_id=values.get('target')
        if target_id is not None and target_id not in goals:raise ProgramRejected('Unknown task goal')
        try:
            request=SkillRequest(call.func.attr,oid,target=None if target_id is None else goals[target_id],
                                 functional_pose_id=values.get('functional_pose'))
        except ValueError as exc:raise ProgramRejected(str(exc)) from exc
        steps.append(request)
    # Validate symbolic holding dependencies, before any simulator/robot call.
    held=world_snapshot['robot']['holding'] if world_snapshot.get('robot') else 'unknown'
    for request in steps:
        if request.skill=='grasp':
            if held!='empty':raise ProgramRejected('A grasp requires known empty gripper in the program state')
            held=request.object_id
        elif request.skill=='place':
            if held!=request.object_id:raise ProgramRejected('Place must follow verified grasp of the same instance')
            held='empty'
        elif request.skill=='push' and held!='empty':
            raise ProgramRejected('Cannot push while holding another object')
    return dict(schema='rm75_atomic_program_v1',source=code,
                based_on_snapshot=world_snapshot['snapshot_id'],goals_digest=digest(goals),
                steps=[r.as_dict() for r in steps],execution_authorized=False)


def run_program(program, runtime, *, maximum_skills=64):
    if program.get('schema')!='rm75_atomic_program_v1' or not 1<=len(program.get('steps',[]))<=maximum_skills:
        raise ProgramRejected('Invalid compiled program')
    # Compiled programs are trusted interpreter output; each skill re-observes.
    # Do not replay already finished skill records as a recovery strategy.
    results=[]
    from .paired_grasp_place import atomic_groups
    index=-1
    for group in atomic_groups(runtime,[SkillRequest(**step) for step in program['steps']]):
        index+=len(group)
        results.extend(outcome.as_dict() for request,outcome in group)
        request,outcome=group[-1]
        if not outcome.skill_verified:
            return dict(completed=False,results=results,next_skill=index,
                        reason='Stop dependencies; high-level replanner must use the new measured SWM',
                        final_snapshot=runtime.world.snapshot())
    return dict(completed=True,results=results,next_skill=None,
                verification_domain=runtime.world.domain,final_snapshot=runtime.world.snapshot())


class SWMAgentPlanner:
    """Use an already configured completion callable; no model/key is hardcoded."""
    def __init__(self, complete):self.complete=complete

    def plan(self, instruction, snapshot, goals, capabilities, *, recovery=None):
        if not isinstance(instruction,str) or not 1<=len(instruction)<=6000:raise ValueError('Invalid task instruction')
        public=dict(objects=[dict(id=k,name=v['name'],asset_id=v['asset_id'],lifecycle=v['lifecycle'],
                                  measured=v['measured'],functional_poses=snapshot['assets'][v['asset_id']].get('functional_poses',[])) for k,v in snapshot['objects'].items()],
                    goals=list(goals),capabilities=capabilities,
                    robot_holding=(snapshot.get('robot') or {}).get('holding','unknown'),
                    recovery=recovery)
        messages=[dict(role='system',content=(
            'Decompose the task and write bounded atomic skill calls. Return ONLY JSON with '
            'keys decomposition (list of short strings) and code (string). Allowed statements: '
            'skills.grasp("instance", functional_pose="registered_id"); '
            'skills.place("instance", target="goal_id"); skills.push/pull/rotate with the same literal fields. '
            'No imports, loops, arbitrary Python, pose literals, safety flags or hardware calls. '
            'Never use a capability that is false. A failed grasp cannot be followed by place. '
            'The runtime, not you, measures state and authorizes execution.')),
            dict(role='user',content=json.dumps(dict(instruction=instruction,swm=public),ensure_ascii=False))]
        output=self.complete(messages)
        proposal=json.loads(output) if isinstance(output,str) else output
        if not isinstance(proposal,dict) or set(proposal)!={'decomposition','code'}:
            raise ProgramRejected('Planner did not return decomposition/code')
        if not isinstance(proposal['decomposition'],list) or not 1<=len(proposal['decomposition'])<=64 or any(not isinstance(x,str) or len(x)>1000 for x in proposal['decomposition']):
            raise ProgramRejected('Malformed bounded decomposition')
        compiled=compile_skill_code(proposal['code'],world_snapshot=snapshot,goals=goals)
        if any(capabilities.get(step['skill']) is not True for step in compiled['steps']):
            raise ProgramRejected('Plan requests an unavailable atomic adapter')
        compiled['decomposition']=copy.deepcopy(proposal['decomposition'])
        return compiled
