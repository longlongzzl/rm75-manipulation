"""Per-job scope for optional diagnostics in a shared three-task profile."""
import copy

PICKPLACE_ONLY = ('audit_current_table_failures', 'tennis_ik_review',
                  'failed_object_ik_seeds', 'render_ik_candidates')


def scoped_profile(spec, profile):
    if not isinstance(spec, dict) or spec.get('task') not in ('pickplace', 'magnetic', 'pusht'):
        raise ValueError('Unknown task for diagnostic scoping')
    result = copy.deepcopy(profile)
    ignored = []
    if spec['task'] != 'pickplace':
        section = result.get('pickplace', {})
        for key in PICKPLACE_ONLY:
            if key in section:
                if section[key]: ignored.append(key)
                section.pop(key)
    # Physics, collision policies, real qualification and source inputs are untouched.
    return result, ignored
