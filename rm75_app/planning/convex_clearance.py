"""Advisory convex-volume separation certificates, not execution permission."""
from __future__ import annotations
import hashlib
import numpy as np


def certify_convex_clearance(first, second, *, margins):
    """Prove a lower distance bound from a separating projection axis.

    Failure to find a sufficient axis is inconclusive, not a penetration claim.
    This helper never overrides collision diagnostics or authorizes motion.
    """
    from scipy.spatial import ConvexHull
    points = [np.asarray(value, dtype=np.float64).copy() for value in (first, second)]
    if any(value.ndim != 2 or value.shape[1] != 3 or not 4 <= len(value) <= 512
           or not np.isfinite(value).all() for value in points):
        raise ValueError('Bounded finite full convex-volume vertices required')
    if not isinstance(margins, dict) or not margins or any(
            not isinstance(name, str) or not name or not np.isscalar(value)
            or not np.isfinite(value) or value < 0 for name, value in margins.items()):
        raise ValueError('Explicit finite nonnegative clearance terms required')
    required = float(sum(margins.values()))
    if not np.isfinite(required):
        raise ValueError('Finite total clearance required')
    hulls = [ConvexHull(value) for value in points]
    if any(hull.volume <= 0 for hull in hulls):
        raise ValueError('Full-dimensional convex collision volumes required')
    axes = np.vstack([hull.equations[:, :3] for hull in hulls])
    if len(axes) > 2048:
        raise ValueError('Separating-axis diagnostic budget exhausted')
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    a, b = (value @ axes.T for value in points)
    forward = b.min(axis=0)-a.max(axis=0)
    backward = a.min(axis=0)-b.max(axis=0)
    gaps = np.maximum(forward, backward)
    index = int(np.argmax(gaps))
    axis = axes[index] if forward[index] >= backward[index] else -axes[index]
    # Inflate projected intervals rather than rounding an optimistic gap up.
    guard = 128 * np.finfo(np.float64).eps * (1 + max(np.max(np.abs(p)) for p in points))
    intervals = [[float(np.min(p @ axis)-guard), float(np.max(p @ axis)+guard)] for p in points]
    lower = intervals[1][0]-intervals[0][1]
    if not np.isfinite(lower):
        raise ValueError('Finite conservative separation bound required')
    return dict(source='full_convex_volume_separating_axis_lower_bound',
        sufficient_clearance_proven=bool(lower > required),
        raw_projected_gap_m=float(gaps[index]), conservative_gap_lower_bound_m=lower,
        required_clearance_m=required, clearance_terms_m=dict(margins),
        residual_clearance_lower_bound_m=lower-required,
        axis=axis.tolist(), outward_projected_intervals_m=intervals,
        numeric_interval_guard_m=float(guard),
        vertex_counts=[len(p) for p in points],
        vertex_sha256=[hashlib.sha256(p.astype('<f8').tobytes()).hexdigest() for p in points],
        exact_minimum_distance=False, failure_means='insufficient_certificate_not_proven_collision',
        execution_authorized=False, policy_override_installed=False)
