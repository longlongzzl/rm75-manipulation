"""Runtime collision-sphere poses for the coupled RM75 gripper."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np


DYNAMIC_GRIPPER_LINKS = (
    "gripper_Left_1_Link",
    "gripper_Left_Support_Link",
    "gripper_Left_2_Link",
    "gripper_Right_1_Link",
    "gripper_Right_Support_Link",
    "gripper_Right_2_Link",
    "left_pad",
    "right_pad",
)


INTERNAL_GRIPPER_LINKS = ("gripper_base_link", *DYNAMIC_GRIPPER_LINKS)


def ignore_gripper_internal_self_collision(kinematics: dict) -> None:
    """Exclude only pairs wholly inside the gripper, preserving every sphere.

    Called on an in-memory robot configuration before native solver creation.
    Arm, payload, and world collision checks are unaffected.
    """
    links = set(INTERNAL_GRIPPER_LINKS)
    missing = links - set(kinematics['collision_link_names'])
    if missing:
        raise ValueError(f'Incomplete gripper collision configuration: {sorted(missing)}')
    ignored = kinematics.setdefault('self_collision_ignore', {})
    for link in INTERNAL_GRIPPER_LINKS:
        ignored[link] = sorted(set(ignored.get(link, ())) | (links - {link}))


def _vector(text: str | None) -> np.ndarray:
    return np.fromstring(text or "0 0 0", sep=" ", dtype=np.float64)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy, dtype=np.float64)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def gripper_link_transforms(
    urdf_path: str | Path, joint_position: float | Mapping[str, float]
) -> dict[str, np.ndarray]:
    """Return gripper-base-relative transforms at one coupled jaw position."""
    root = ET.parse(Path(urdf_path)).getroot()
    children: dict[str, list[tuple[str, ET.Element]]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        children.setdefault(str(parent.get("link")), []).append(
            (str(child.get("link")), joint)
        )
    transforms = {"gripper_base_link": np.eye(4, dtype=np.float64)}
    stack = ["gripper_base_link"]
    while stack:
        parent_name = stack.pop()
        for child_name, joint in children.get(parent_name, ()):
            origin = joint.find("origin")
            local = np.eye(4, dtype=np.float64)
            if origin is not None:
                local[:3, 3] = _vector(origin.get("xyz"))
                local[:3, :3] = _rpy_matrix(_vector(origin.get("rpy")))
            if joint.get("type") not in {"fixed", "floating"}:
                axis_element = joint.find("axis")
                axis = _vector(
                    "1 0 0" if axis_element is None else axis_element.get("xyz")
                )
                rotation = np.eye(4, dtype=np.float64)
                angle=float(joint_position[joint.get('name')]) if isinstance(joint_position,Mapping) else float(joint_position)
                if not np.isfinite(angle):raise ValueError('Nonfinite gripper joint position')
                rotation[:3, :3] = _axis_angle_matrix(axis, angle)
                local = local @ rotation
            transforms[child_name] = transforms[parent_name] @ local
            stack.append(child_name)
    return transforms



def sphere_pair_penetration_m(first, second, padding_first=0., padding_second=0.):
    """Linear overlap in metres, including the original self-collision padding.

    cuRobo2's stored pair score is (r1+r2)^2 - distance^2, in square metres.
    Its sign detects collision but the value is not a penetration distance.
    """
    a=np.asarray(first,dtype=float).reshape(4)
    b=np.asarray(second,dtype=float).reshape(4)
    if not np.isfinite([*a,*b,padding_first,padding_second]).all():
        raise ValueError('Nonfinite sphere pair')
    return float(a[3]+b[3]+padding_first+padding_second-np.linalg.norm(a[:3]-b[:3]))


def remap_link_sphere_centers(
    centers: np.ndarray,
    reference_link_transform: np.ndarray,
    desired_link_transform: np.ndarray,
) -> np.ndarray:
    """Express desired-pose sphere centers in the locked link local frame."""
    values = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
    correction = np.linalg.inv(reference_link_transform) @ desired_link_transform
    homogeneous = np.concatenate([values, np.ones((len(values), 1))], axis=1)
    return (correction @ homogeneous.T).T[:, :3]


class DynamicGripperSphereController:
    """Mutate one sphere set to follow the commanded open/closed jaw pose."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        reference_joint_position: float = 0.6,
        open_joint_position: float = 0.0,
        closed_joint_position: float = 0.6,
    ) -> None:
        self.reference_transforms = gripper_link_transforms(
            urdf_path, reference_joint_position
        )
        self.state_transforms = {
            "open": gripper_link_transforms(urdf_path, open_joint_position),
            "closed": gripper_link_transforms(urdf_path, closed_joint_position),
        }
        self._reference_centers: dict[int, dict[str, np.ndarray]] = {}

    def apply(self, kinematics_config: Any, state: str) -> None:
        if state not in self.state_transforms:
            raise ValueError(f"unsupported gripper collision state: {state}")
        parameter_id = id(kinematics_config)
        reference = self._reference_centers.setdefault(parameter_id, {})
        for link_name in DYNAMIC_GRIPPER_LINKS:
            indices = kinematics_config.get_sphere_index_from_link_name(link_name)
            if len(indices) == 0:
                continue
            spheres = kinematics_config.link_spheres[:, indices, :]
            if link_name not in reference:
                reference[link_name] = (
                    spheres[0, :, :3].detach().cpu().numpy().astype(np.float64)
                )
            mapped = remap_link_sphere_centers(
                reference[link_name],
                self.reference_transforms[link_name],
                self.state_transforms[state][link_name],
            )
            kinematics_config.link_spheres[:, indices, :3] = (
                spheres.new_tensor(mapped).unsqueeze(0)
            )
