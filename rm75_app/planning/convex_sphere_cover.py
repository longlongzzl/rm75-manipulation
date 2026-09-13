"""Bounded conservative convex-volume sphere cover, not automatic deployment.

A convex hull is partitioned into centroid-to-face tetrahedra. Longest-edge
bisection preserves each tetrahedron's volume. Every emitted float32 sphere
contains all four vertices of its leaf tetrahedron and hence its entire volume.
Halfspace excess is a tightness bound, not an Euclidean Hausdorff claim.
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


def _bound(tetra, planes):
    center = tetra.mean(axis=0).astype(np.float32).astype(float)
    distances = np.linalg.norm(tetra-center, axis=1)
    guard = 64 * np.finfo(float).eps * (1 + np.max(np.abs(tetra)))
    radius = float(np.nextafter(np.float32(max(distances)+guard), np.float32(np.inf)))
    excess = float(np.max(planes[:, :3] @ center + planes[:, 3] + radius))
    return center, radius, excess


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
    for face in hull.simplices:
        tetra = np.vstack((center, vertices[face]))
        sphere = _bound(tetra, hull.equations)
        heapq.heappush(queue, (-sphere[2], serial, tetra, sphere))
        serial += 1
    if len(queue) > max_spheres:
        raise SphereCoverBudgetExceeded(dict(completed=False, minimum_root_count=len(queue),
            max_spheres=max_spheres, deployable=False))
    while queue:
        _, _, tetra, sphere = heapq.heappop(queue)
        if sphere[2] <= max_halfspace_excess_m:
            leaves.append((tetra, sphere))
            continue
        if len(queue) + len(leaves) + 2 > max_spheres:
            raise SphereCoverBudgetExceeded(dict(completed=False, max_spheres=max_spheres,
                current_partition_count=len(queue)+len(leaves)+1,
                worst_halfspace_excess_m=sphere[2],
                required_halfspace_excess_m=max_halfspace_excess_m, deployable=False))
        pairs = [(i, j) for i in range(4) for j in range(i+1, 4)]
        a, b = max(pairs, key=lambda pair: float(np.sum((tetra[pair[0]]-tetra[pair[1]])**2)))
        midpoint = (tetra[a]+tetra[b])/2.
        for index in (a, b):
            child = tetra.copy()
            child[index] = midpoint
            bound = _bound(child, hull.equations)
            heapq.heappush(queue, (-bound[2], serial, child, bound))
            serial += 1
    covered_volume = sum(_volume(tetra) for tetra, _ in leaves)
    if not np.isclose(covered_volume, hull.volume, atol=1e-15, rtol=1e-9):
        raise ValueError('Tetrahedron partition volume is inconsistent')
    residual = max(float(np.max(np.linalg.norm(tetra-bound[0], axis=1)-bound[1]))
                   for tetra, bound in leaves)
    if residual > 0:
        raise ValueError('Float32 sphere does not contain its full leaf tetrahedron')
    return dict(source='convex_tetrahedron_partition_sphere_cover', completed=True,
        deployable=False, deployment_reason='Native capacity and consumer binding not validated',
        sphere_count=len(leaves), max_spheres=max_spheres,
        spheres=[dict(center=bound[0].tolist(), radius=bound[1]) for _, bound in leaves],
        coverage=dict(method='all_leaf_tetra_vertices_inside_convex_float32_balls',
            original_hull_volume_m3=float(hull.volume), leaf_volume_sum_m3=covered_volume,
            maximum_vertex_outside_ball_m=residual,
            maximum_halfspace_excess_m=max(bound[2] for _, bound in leaves),
            required_halfspace_excess_m=max_halfspace_excess_m,
            full_convex_volume_covered=True, surface_sampling_only=False),
        original_collision_buffers_changed=False)
