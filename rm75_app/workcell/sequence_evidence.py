"""Reconcile UI progress with the actual FrozenWorldValidation result contract.

This module never turns per-object progress into physical task verification.
The obsolete key is display-only compatibility for earlier client fixtures.
"""
from collections import Counter


def summarize_sequence(result, requested):
    requested = list(requested)
    if (not requested or any(not isinstance(name, str) or not name for name in requested)
            or len(set(requested)) != len(requested)):
        raise ValueError('Sequence summary needs distinct requested source names')
    key = ('frozen_world_validation' if 'frozen_world_validation' in result
           else 'native_full_world_validation')
    canonical = key == 'frozen_world_validation'
    data = result.get(key, {})
    if not isinstance(data, dict):
        data = {}
    rows = data.get('source_outcomes', [])
    if not isinstance(rows, list):
        rows = []
    counts = Counter()
    malformed = 0
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get('source'), str):
            malformed += 1
            continue
        # Actual producer supplies the flag explicitly; missing is not evidence.
        not_prefetch = (row.get('prefetch_capture_only') is False if canonical
                        else row.get('prefetch_capture_only', False) is False)
        if row.get('success') is True and row.get('foreground') is True and not_prefetch:
            counts[row['source']] += 1
    completed = [name for name in requested if counts[name] > 0]
    source_match = data.get('requested_sources') == requested
    gates = ('passed', 'input_unchanged', 'requested_sources_completed',
             'all_sources_transport_observed', 'independent_clearance_passed')
    verified = bool(canonical and result.get('command_success') is True
                    and source_match and malformed == 0
                    and counts == Counter(requested)
                    and all(data.get(name) is True for name in gates))
    return dict(requested=requested, native_completed=completed,
        pending=[name for name in requested if name not in completed], total=len(requested),
        completed_count=len(completed), same_scene_native_sequence=bool(canonical and source_match),
        all_requested_verified=verified, evidence_key=key if data else None,
        evidence_is_native_contract=canonical and bool(data),
        duplicate_success_sources={name: count for name, count in counts.items() if count > 1},
        unexpected_success_sources=sorted(set(counts)-set(requested)),
        ignored_malformed_rows=malformed,
        independent_physical_success=None, partial_results_are_not_task_success=True)
