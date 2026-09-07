"""Classify the reviewed native execution labels, not the planner's path type."""


def guarded_stage(label):
    label = str(label)
    return (label.startswith('post_place_clearance') or
            'return_to_cycle_start' in label or 'return_to_start' in label)


def stage_kind(label):
    return {
        'post_place_clearance': 'release_only',
        'return_to_cycle_start_prelift': 'release_only',
        'post_place_clearance_return_to_cycle_start': 'release_then_return',
        'return_to_cycle_start': 'return_only',
        'post_place_direct_return_to_cycle_start': 'return_only',
    }.get(str(label))
