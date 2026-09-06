"""Frame math. Metres, right-handed transforms; never silently fix reflection."""
from __future__ import annotations
import numpy as np


def vector(value, n: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (n,) or not np.isfinite(result).all():
        raise ValueError(f'{name}: expected {n} finite numbers')
    return result


def rigid(value, name='transform') -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ValueError(f'{name}: expected finite 4x4 matrix')
    r = result[:3, :3]
    if not np.allclose(result[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError(f'{name}: invalid homogeneous row')
    if not np.allclose(r.T @ r, np.eye(3), atol=2e-4) or not np.isclose(np.linalg.det(r), 1, atol=2e-4):
        raise ValueError(f'{name}: rotation is not in SO(3)')
    return result.copy()


def rotation_error(a, b) -> float:
    r = np.asarray(a).T @ np.asarray(b)
    return float(np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1)))


def quaternion_matrix(q) -> np.ndarray:
    w, x, y, z = vector(q, 4, 'quaternion_wxyz')
    norm = float(np.linalg.norm([w, x, y, z]))
    if norm < 1e-9:
        raise ValueError('zero quaternion')
    w, x, y, z = np.array([w, x, y, z]) / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix_quaternion(r) -> list[float]:
    # Eigenvector formulation is stable at pi and avoids a scipy runtime dependency.
    r = np.asarray(r, dtype=float)
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3
    _, v = np.linalg.eigh(k)
    q = v[:, -1][[3,0,1,2]]
    return (q if q[0] >= 0 else -q).tolist()
