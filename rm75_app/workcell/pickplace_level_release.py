"""Restore the requested level preplace/release constraint for Tennis.

Keep the archived generators and all seeds/world checks. Level is a task pose
constraint, not a scoring preference with a tilted fallback. No Jimu/insert rule
is changed. Only original poses are eligible; do not rotate an attached object.
"""
import functools
import inspect
import numpy as np

from .transforms import quaternion_matrix


def level_pose(pose, flatten):
    q = np.asarray(flatten(pose.q), dtype=float).reshape(-1)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        return False
    r = quaternion_matrix(q / np.linalg.norm(q))
    # Numerical equality for GENERATED downward poses (not a relaxed IK error).
    return bool(r[2, 2] < 0 and np.max(np.abs(r[2, :2])) <= 1e-5)


def filter_candidates(candidates, flatten):
    kept = []
    for candidate in candidates:
        poses = [candidate[key] for key in ('pose', 'pre_place_pose', 'hover_pose',
                                            'place_pose', 'release_pose') if key in candidate]
        if ('pose' in candidate and 'place_pose' in candidate
                and all(pose is not None and level_pose(pose, flatten) for pose in poses)):
            kept.append(candidate)
    return kept


def install(direct, emit):
    originals = []
    for name in ('_build_direct_pre_place_candidates', '_build_direct_place_candidates'):
        original = getattr(direct, name)
        signature = inspect.signature(original)
        def wrap(original, signature, name):
            @functools.wraps(original)
            def build(*args, **kwargs):
                bound = signature.bind(*args, **kwargs).arguments
                result = original(*args, **kwargs)
                if direct._current_source_object_name(bound['args']) != 'tennis':
                    return result
                kept = filter_candidates(result, direct.targeted.base.flatten_np)
                emit(dict(event='pickplace_level_release_constraint', source='tennis',
                          generator=name, original_count=len(result), eligible_count=len(kept),
                          tilted_fallback_allowed=False, poses_modified=False, seeds_modified=False))
                print(f'[level release] tennis {name}: {len(kept)}/{len(result)} original poses satisfy '
                      'level preplace AND place; tilted fallback disabled')
                return kept
            return build
        originals.append((name, original))
        setattr(direct, name, wrap(original, signature, name))
    def close():
        for name, original in originals:
            setattr(direct, name, original)
    return close
