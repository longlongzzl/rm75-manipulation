"""Bounded paired-endpoint continuation for unchanged native PickPlace goals.

An accepted hover can seed its own missing release, or vice versa. This does
NOT create a new grasp, move a goal, change the payload, suppress a collision,
or turn endpoint success into a complete motion-chain success.
"""
from __future__ import annotations
from contextvars import ContextVar
from collections import OrderedDict
import copy
import functools
import time
import numpy as np
from .io import digest, finite, integer


class PairRepairStateError(BaseException):
    pass


def freeze_batch(rows):
    """Clone shared native result buffers once, before another solver call."""
    raw_copies = {}; result = []
    for row in rows:
        item = copy.copy(row)
        raw = getattr(row, 'raw_result', None)
        if raw is not None:
            if id(raw) not in raw_copies:
                saved = copy.copy(raw)
                for name in ('solution', 'success', 'position_error', 'rotation_error'):
                    value = getattr(raw, name, None)
                    if value is not None:
                        setattr(saved, name, value.detach().clone() if hasattr(value, 'detach') else
                                value.copy() if hasattr(value, 'copy') else copy.deepcopy(value))
                raw_copies[id(raw)] = saved
            item.raw_result = raw_copies[id(raw)]
        if getattr(row, 'goal_joint', None) is not None:
            item.goal_joint = np.asarray(row.goal_joint).copy()
        item.debug = copy.deepcopy(getattr(row, 'debug', None) or {})
        result.append(item)
    return result


def complete_pairs(rows, starts, goals, query, validate, *, limit=12, deadline=None,
                   clock=time.monotonic, seen=None, identity=lambda i: i, check=lambda: None):
    """Pure dependency-injected repair; goal order is [all hovers, all releases]."""
    if len(rows) != len(starts) or len(rows) != len(goals) or len(rows) % 2:
        raise ValueError('Paired native goal/result identity mismatch')
    n = len(rows)//2; result=freeze_batch(rows); calls=0; records=[]
    seen = set() if seen is None else seen
    for index in range(n):
        a, b = result[index], result[index+n]
        a_ok = bool(a.success) and a.goal_joint is not None
        b_ok = bool(b.success) and b.goal_joint is not None
        if a_ok == b_ok: continue
        target = index+n if a_ok else index; source = index if a_ok else index+n
        key = identity(target)
        if key in seen: continue
        if calls >= limit or deadline is not None and clock() >= deadline: break
        check(); seen.add(key)
        # Preserve all native successes even if later queries reuse their buffers.
        seed=np.asarray(result[source].goal_joint).copy()
        candidate=freeze_batch([query(seed, goals[target])])[0]; calls+=1
        accepted=(bool(candidate.success) and candidate.goal_joint is not None and
                  bool(validate(candidate.goal_joint)))
        records.append(dict(pair=index, goal_index=target, seeded_from=source, accepted=accepted))
        if accepted:
            candidate.debug.update(paired_endpoint_repair=True, original_goal_index=target,
                                   ik_search_goal_count=1, endpoint_only=True)
            result[target]=candidate
    return result, dict(query_calls=calls, newly_completed=sum(x['accepted'] for x in records),
                        repairs=records, goals_modified=False, full_chain_verified=False)


def install(direct, emit, stop, *, max_queries=12, budget_s=5.):
    integer(max_queries, 'max_queries', 1, 32); finite(budget_s, 'budget_s', .1, 15)
    from .pickplace_lift_diagnostics import _state, _configuration_evidence
    original_evaluate=direct._fast_chain_evaluate_paired_relation_records
    original_batch=direct._profile_fast_chain_solve_batch_start_goal_ik
    context=ContextVar('paired_endpoint_repair', default=None)
    cache=OrderedDict()

    @functools.wraps(original_evaluate)
    def evaluate(planner, demo, args, records, start_q, **kwargs):
        token=context.set((args, len(records)))
        try: return original_evaluate(planner, demo, args, records, start_q, **kwargs)
        finally: context.reset(token)

    @functools.wraps(original_batch)
    def batch(args, planner, starts, goals, *, num_seeds):
        with direct._CUROBO_GPU_LOCK:
            rows=original_batch(args, planner, starts, goals, num_seeds=num_seeds)
            scope=context.get()
            source=direct._current_source_object_name(args)
            if (scope is None or scope[0] is not args or source not in ('gluestick', 'hongshupian', 'tennis')
                    or getattr(args, '_planning_prefetch_capture_only', False)):
                return rows
            if getattr(args, 'execute_real', False):
                raise PairRepairStateError('Paired-endpoint repair is frozen-SIM only until separately qualified')
            if len(goals) != 2*scope[1]:
                raise PairRepairStateError('Native paired-goal contract changed')
            before=_state(planner); key=(source, digest(before))
            if key not in cache:
                cache[key]=dict(seen=set(), used=0, spent=0.)
                while len(cache)>16: cache.popitem(last=False)
            state=cache[key]
            if state['used'] >= max_queries or state['spent'] >= budget_s: return rows
            started=time.monotonic()
            def identity(index):
                p,q=planner._extract_pose_components(goals[index])
                return digest(dict(goal_p=np.asarray(p).tolist(), goal_q=np.asarray(q).tolist(),
                                   seed_source=np.asarray(starts[index]).tolist()))
            def query(seed, goal):
                stop.check()
                result=planner.solve_batch_start_goal_ik([seed], [goal], num_seeds=num_seeds,
                                                         use_cuda_graph_batch=False)
                if len(result)!=1: raise PairRepairStateError('Native singleton IK returned another goal count')
                return result[0]
            try:
                repaired, evidence=complete_pairs(rows, starts, goals, query,
                    lambda q: _configuration_evidence(planner,q)['valid'],
                    limit=max_queries-state['used'], deadline=started+budget_s-state['spent'],
                    seen=state['seen'], identity=identity, check=stop.check)
            finally:
                state['spent']+=time.monotonic()-started
                if _state(planner)!=before:
                    raise PairRepairStateError('Paired-endpoint continuation changed collision/attachment state')
            state['used']+=evidence['query_calls']
            emit(dict(event='paired_endpoint_repair', source=source, original_seeds_per_query=num_seeds,
                added_solve_elapsed_s=time.monotonic()-started, cumulative_queries=state['used'],
                cumulative_added_solve_s=state['spent'], state_unchanged=True,
                budget_is_soft_between_queries=True, **evidence))
            return repaired

    direct._fast_chain_evaluate_paired_relation_records=evaluate
    direct._profile_fast_chain_solve_batch_start_goal_ik=batch
    def close():
        direct._fast_chain_evaluate_paired_relation_records=original_evaluate
        direct._profile_fast_chain_solve_batch_start_goal_ik=original_batch
    return close
