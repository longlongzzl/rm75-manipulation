"""Validate the *existing* Jimu builder format without rewriting its semantics.

Legacy source: triangle_roof_apriltag_portable._builder_piece_matrix and
_builder_floor_relative_piece_matrix. Builder is Y-up, matrix columns [u,n,v].
Only small export-rounding errors are orthogonalized. Original JSON is retained.
This is a design sanity check, not proof of physical magnetic attachment.
"""
from __future__ import annotations
import copy
from collections import deque
from dataclasses import dataclass
import numpy as np
from rm75_app.workcell.transforms import rigid, vector
from rm75_app.workcell.io import finite

DIMENSIONS = {'square': (.074, .0065, .074),
              'half_square': (.037, .0065, .074),
              'triangle': (.074, .0065, .135)}


def piece_key(p):
    return str(p.get('role') or p.get('id') or '').strip()


def piece_matrix(p):
    center = vector(p.get('center'), 3, 'center')
    r = np.column_stack([vector(p.get(k), 3, k) for k in ('u', 'n', 'v')])
    if np.max(np.abs(r.T @ r - np.eye(3))) > .02 or np.linalg.det(r) < .98:
        raise ValueError(f'{piece_key(p)}: invalid/reflected [u,n,v] axes')
    x = r[:, 0] / np.linalg.norm(r[:, 0])
    y = r[:, 1] - np.dot(r[:, 1], x) * x
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    t = np.eye(4)
    t[:3, :3] = np.column_stack([x, y, z])
    t[:3, 3] = center
    return t


def parent_key(p):
    parent = p.get('parentRole', p.get('parent_role', p.get('parentId', p.get('parent_id', p.get('parent')))))
    if isinstance(parent, dict):
        parent = parent.get('role', parent.get('id'))
    return str(parent).strip() if parent else None


def bounds(p):
    t = piece_matrix(p)
    dims = np.array(DIMENSIONS[p['type']])
    # Broad-phase bounds only; triangle bounding boxes are not collision meshes.
    half_world = abs(t[:3, :3]) @ (dims / 2)
    return t[:3, 3] - half_world, t[:3, 3] + half_world


@dataclass(frozen=True)
class Design:
    payload: dict
    ordered_roles: tuple[str, ...]
    targets_builder: dict[str, np.ndarray]
    warnings: tuple[str, ...]

    def report(self):
        return {'schema': self.payload['schema'], 'piece_count': len(self.payload['pieces']),
                'movable_count': len(self.ordered_roles), 'ordered_roles': list(self.ordered_roles),
                'frame': 'builder_y_up_columns_u_n_v', 'warnings': list(self.warnings),
                'validation': 'design_only_not_physical_attachment',
                'targets_builder': {k: v.tolist() for k,v in self.targets_builder.items()}}


def validate_design(payload, *, max_movable=12) -> Design:
    if not isinstance(payload, dict) or payload.get('schema') != 'jimu_builder_scene_v1':
        raise ValueError('Expected existing jimu_builder_scene_v1 format')
    pieces = payload.get('pieces')
    if not isinstance(pieces, list) or not 1 <= len(pieces) <= 64:
        raise ValueError('pieces must contain 1..64 entries including fixed supports')
    by_key, aliases, targets = {}, {}, {}
    for p in pieces:
        if not isinstance(p, dict) or p.get('type') not in DIMENSIONS:
            raise ValueError('Piece must be square, half_square or triangle')
        key = piece_key(p)
        if not key or len(key) > 100 or key in by_key:
            raise ValueError('Piece roles must be nonempty and unique')
        if type(p.get('locked', False)) is not bool:
            raise ValueError('locked must be boolean')
        by_key[key] = p
        targets[key] = piece_matrix(p)
        if abs(targets[key][:3,3]).max() > 3:
            raise ValueError('Builder coordinates must be in metres (outside 3m sanity bound)')
        for alias in (key, str(p.get('id') or key)):
            if alias in aliases and aliases[alias] != key:
                raise ValueError('Ambiguous id/role alias')
            aliases[alias] = key
    moving = [k for k,p in by_key.items() if not p.get('locked', False)]
    if not 1 <= len(moving) <= max_movable:
        raise ValueError(f'Expected 1..{max_movable} movable pieces; locked base is not counted')
    incoming = {k: 0 for k in by_key}
    children = {k: [] for k in by_key}
    warnings = []
    for key,p in by_key.items():
        parent = parent_key(p)
        if parent:
            if parent not in aliases or aliases[parent] == key:
                raise ValueError(f'{key}: missing or self parent {parent}')
            parent = aliases[parent]
            incoming[key] += 1
            children[parent].append(key)
            for field in ('parentRelativeTransform','parent_relative_transform','T_parent_piece'):
                if field in p:
                    relative = rigid(p[field], field)
                    if not np.allclose(targets[parent] @ relative, targets[key], atol=.001):
                        raise ValueError(f'{key}: parent-relative pose disagrees with exported target')
                    break
        elif not p.get('locked', False):
            warnings.append(f'{key}: no explicit parent; legacy role/geometry planner resolves support')
        lo,_ = bounds(p)
        if lo[1] < -.005:
            warnings.append(f'{key}: Y-up bounding box crosses the table; inspect mesh/contact geometry')
    queue = deque(k for k in by_key if incoming[k] == 0)
    order = []
    while queue:
        k = queue.popleft()
        order.append(k)
        for child in children[k]:
            incoming[child] -= 1
            if incoming[child] == 0:
                queue.append(child)
    if len(order) != len(by_key):
        raise ValueError('Assembly dependency graph contains a cycle')
    # Coincident parts are certainly wrong; OBB overlap alone is not a reliable
    # rejection criterion for the concave/triangle magnetic assembly geometry.
    keys = list(by_key)
    for i,a in enumerate(keys):
        for b in keys[i+1:]:
            if np.linalg.norm(targets[a][:3,3]-targets[b][:3,3]) < .0001:
                raise ValueError(f'Coincident piece centers: {a}, {b}')
    return Design(copy.deepcopy(payload), tuple(k for k in order if k in moving), targets, tuple(warnings))


def world_targets(design: Design, T_world_floor, floor_thickness=.0065):
    floor = rigid(T_world_floor, 'T_world_floor')
    thickness = finite(floor_thickness, 'floor_thickness', .001, .1)
    result = {}
    for key, t in design.targets_builder.items():
        relative = t.copy()
        relative[:3, 3] -= [0, thickness / 2, 0]
        result[key] = floor @ relative
    return result
