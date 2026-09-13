"""Bounded reusable spheres with complete convex-volume partition evidence.

Every leaf tetrahedron is contained in one ball; balls may cover many leaves.
The hull-face fan and longest-edge bisections preserve the covered volume.
Halfspace excess is not an Euclidean Hausdorff distance or deployment approval.
"""
from __future__ import annotations
import heapq
import numpy as np


class SphereCoverBudgetExceeded(ValueError):
    def __init__(self, evidence):
        super().__init__('Conservative sphere cover exceeded frozen sphere budget')
        self.evidence = evidence


def _volume(tetra):
    return abs(float(np.linalg.det(tetra[1:] - tetra[0]))) / 6.


def _inscribed_ball(center, planes, excess):
    center = np.asarray(center, dtype=np.float32).astype(float)
    guard = 64 * np.finfo(float).eps * (1 + np.max(np.abs(center)))
    radius = float(np.nextafter(np.float32(np.min(
        excess - planes[:, :3] @ center - planes[:, 3]) - guard), np.float32(-np.inf)))
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError('No positive finite conservative ball at candidate center')
    return center, radius


def cover_convex_volume(vertices, *, max_halfspace_excess_m, max_spheres=8192):
    from scipy.spatial import ConvexHull
    vertices = np.asarray(vertices, dtype=float)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 4
            or not np.isfinite(vertices).all() or type(max_spheres) is not int
            or not 4 <= max_spheres <= 8192 or not np.isfinite(max_halfspace_excess_m)
            or not 0 < max_halfspace_excess_m <= .01):
        raise ValueError('Finite convex geometry, positive tightness and bounded capacity required')
    hull = ConvexHull(vertices)
    if hull.volume <= 0:
        raise ValueError('Positive convex collision volume required')
    center = vertices[hull.vertices].mean(axis=0)
    queue, leaves = [], []
    serial = 0
    balls = [_inscribed_ball(center, hull.equations, max_halfspace_excess_m)]
    for face in hull.simplices:
        tetra = np.vstack((center, vertices[face]))
        heapq.heappush(queue, (-_volume(tetra), serial, tetra))
        serial += 1

    def fail(reason):
        raise SphereCoverBudgetExceeded(dict(completed=False, reason=reason,
            max_spheres=max_spheres, candidate_spheres=len(balls),
            current_partition_count=len(queue)+len(leaves)+1,
            required_halfspace_excess_m=max_halfspace_excess_m, deployable=False))

    if len(queue) > max_spheres:
        fail('initial hull partition capacity')
    while queue:
        _, _, tetra = heapq.heappop(queue)
        centers = np.asarray([ball[0] for ball in balls])
        radii = np.asarray([ball[1] for ball in balls])
        distances = np.linalg.norm(tetra[None, :, :] - centers[:, None, :], axis=2)
        covered = np.flatnonzero(np.max(distances, axis=1) <= radii)
        if len(covered):
            leaves.append((tetra, int(covered[0])))
            continue
        # Shift toward the hull interior to reuse larger balls while keeping
        # each ball inside every expanded hull halfspace. No radius shrinking
        # is accepted as a replacement for full tetrahedron coverage.
        origin = tetra.mean(axis=0)
        candidates = [_inscribed_ball((1-t)*origin+t*center, hull.equations,
                                      max_halfspace_excess_m) for t in np.linspace(0., 1., 9)]
        candidate = max(candidates, key=lambda ball: ball[1] /
                        max(float(np.max(np.linalg.norm(tetra-ball[0], axis=1))), 1e-30))
        if not any(np.array_equal(candidate[0], ball[0]) and candidate[1] == ball[1]
                   for ball in balls):
            if len(balls) >= max_spheres:
                fail('sphere capacity')
            balls.append(candidate)
        selected = next(i for i, ball in enumerate(balls)
                        if np.array_equal(candidate[0], ball[0]) and candidate[1] == ball[1])
        if np.max(np.linalg.norm(tetra-candidate[0], axis=1)) <= candidate[1]:
            leaves.append((tetra, selected))
            continue
        if len(queue) + len(leaves) + 2 > max_spheres:
            fail('proof partition capacity')
        pairs = [(i, j) for i in range(4) for j in range(i+1, 4)]
        a, b = max(pairs, key=lambda pair: float(np.sum((tetra[pair[0]]-tetra[pair[1]])**2)))
        midpoint = (tetra[a]+tetra[b])/2.
        for index in (a, b):
            child = tetra.copy()
            child[index] = midpoint
            heapq.heappush(queue, (-_volume(child), serial, child))
            serial += 1
    covered_volume = sum(_volume(tetra) for tetra, _ in leaves)
    if not np.isclose(covered_volume, hull.volume, atol=1e-15, rtol=1e-9):
        raise ValueError('Tetrahedron partition volume is inconsistent')
    residual = max(float(np.max(np.linalg.norm(tetra-balls[index][0], axis=1)-balls[index][1]))
                   for tetra, index in leaves)
    used = [balls[index] for index in sorted({index for _, index in leaves})]
    excess = max(float(np.max(hull.equations[:, :3] @ ball[0]+hull.equations[:, 3]+ball[1]))
                 for ball in used)
    if residual > 0 or excess > max_halfspace_excess_m:
        raise ValueError('Float32 coverage or tightness certificate failed')
    return dict(source='convex_tetrahedron_partition_sphere_cover', completed=True,
        deployable=False, deployment_reason='Native capacity and consumer binding not validated',
        sphere_count=len(used), candidate_sphere_count=len(balls), proof_leaf_count=len(leaves),
        max_spheres=max_spheres,
        spheres=[dict(center=ball[0].tolist(), radius=ball[1]) for ball in used],
        coverage=dict(method='all_leaf_tetra_vertices_inside_reused_convex_float32_balls',
            original_hull_volume_m3=float(hull.volume), leaf_volume_sum_m3=covered_volume,
            maximum_vertex_outside_ball_m=residual, maximum_halfspace_excess_m=excess,
            required_halfspace_excess_m=max_halfspace_excess_m,
            full_convex_volume_covered=True, surface_sampling_only=False),
        original_collision_buffers_changed=False)
