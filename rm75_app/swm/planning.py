"""Bounded parallel solving and cross-hypothesis path validation.

The solver factory constructs private native contexts; it never receives a real
executor. Plan generation and physical parameter identification are two distinct
loops: this loop searches FUTURE actions; identification replays ONE past action.
"""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
import copy
from dataclasses import replace
from .scene import SceneInvalid, digest
from .skills import PlannedSkill
from .identification import PhysicsParameters


class ParallelHypothesisPlanner:
    def __init__(self, factory, *, workers=1, max_candidates=4, check=lambda:None):
        if type(workers) is not int or not 1<=workers<=4:raise ValueError('Native concurrency must be bounded')
        if type(max_candidates) is not int or not 1<=max_candidates<=8:raise ValueError('Bounded candidate budget required')
        self.factory=factory;self.workers=workers;self.max_candidates=max_candidates;self.check=check

    def solve_managed(self, request, snapshot, manager):
        bank=manager.planning_hypotheses(request.object_id,snapshot)
        plan,evidence=self.solve(request,snapshot,bank['hypotheses'])
        current=manager.planning_hypotheses(request.object_id,snapshot)
        if digest(current)!=digest(bank):
            raise SceneInvalid('Physical uncertainty changed during candidate planning')
        evidence.update(physics_revision=bank['physics_revision'],belief_digest=bank['belief_digest'],
            posterior_transition_digest=bank['posterior_transition_digest'],
            hypothesis_bank_digest=digest(bank['hypotheses']),plan_id=plan.payload_digest)
        manager.emit(kind='swm_physics_planning_consumed',object_id=request.object_id,
            snapshot_id=snapshot['snapshot_id'],physics_revision=bank['physics_revision'],
            belief_digest=bank['belief_digest'],posterior_transition_digest=bank['posterior_transition_digest'],
            hypothesis_bank_digest=evidence['hypothesis_bank_digest'],plan_id=plan.payload_digest)
        return plan,evidence

    def solve(self, request, snapshot, hypotheses):
        if not snapshot['valid']:raise SceneInvalid('Cannot plan from an invalid SWM')
        if not 1<=len(hypotheses)<=32:raise ValueError('Hypothesis count outside native planning budget')
        ids=[h['id'] for h in hypotheses]
        if len(set(ids))!=len(ids):raise ValueError('Duplicate parameter hypothesis')
        for h in hypotheses:PhysicsParameters(**h['parameters'])
        def solve_one(h):
            self.check();solver=self.factory()
            try:
                if getattr(solver,'owns_real_executor',True):raise PermissionError('Planning workers may not own a real executor')
                generate=getattr(solver,'solve_candidates',None)
                plans=(generate(request,copy.deepcopy(snapshot),copy.deepcopy(h['parameters'])) if generate is not None
                       else [solver.solve(request,copy.deepcopy(snapshot),copy.deepcopy(h['parameters']))])
                if not isinstance(plans,(list,tuple)) or len(plans)>self.max_candidates:
                    raise SceneInvalid('Native candidate batch exceeds the declared planning budget')
                for plan in plans:
                    if plan is not None and (not isinstance(plan,PlannedSkill) or plan.source_snapshot_id!=snapshot['snapshot_id'] or plan.skill_digest!=digest(request.as_dict())):
                        raise SceneInvalid('Solver returned a mismatched atomic plan')
                return [plan for plan in plans if plan is not None]
            finally:solver.close()
        with ThreadPoolExecutor(max_workers=self.workers) as pool:
            plans=[plan for batch in pool.map(solve_one,hypotheses) for plan in batch]
        unique={p.payload_digest:p for p in plans if p is not None}
        candidates=list(unique.values())[:self.max_candidates]
        if not candidates:raise RuntimeError('No native atomic path candidate')
        def evaluate(pair):
            plan,h=pair;self.check();solver=self.factory()
            try:
                if getattr(solver,'owns_real_executor',True):raise PermissionError('Auditors may not own a real executor')
                row=solver.evaluate(plan,copy.deepcopy(snapshot),copy.deepcopy(h['parameters']))
                if row.get('snapshot_id')!=snapshot['snapshot_id'] or row.get('payload_digest')!=plan.payload_digest:
                    raise SceneInvalid('Cross-hypothesis audit is stale or for a different path')
                return row
            finally:solver.close()
        import math
        ranked=[];evidence=[]
        for plan in candidates:
            with ThreadPoolExecutor(max_workers=self.workers) as pool:
                rows=list(pool.map(evaluate,[(plan,h) for h in hypotheses]))
            feasible=all(r.get('feasible') is True and math.isfinite(r.get('cost',float('inf'))) for r in rows)
            evidence.append(dict(payload_digest=plan.payload_digest,feasible=feasible,
                                 hypotheses=dict(zip(ids,rows))))
            if feasible:
                worst=max(rows,key=lambda row:row['cost'])
                if 'expected_object_pose' in worst:
                    plan=replace(plan,expected_object_pose=worst['expected_object_pose'])
                ranked.append((worst['cost'],plan))
        if not ranked:raise RuntimeError('No candidate survives all retained physical hypotheses')
        return min(ranked,key=lambda x:x[0])[1],dict(snapshot_id=snapshot['snapshot_id'],
            generated_candidates=len(unique),evaluated_candidates=len(candidates),
            required_hypotheses=ids,results=evidence,real_execution=False)
