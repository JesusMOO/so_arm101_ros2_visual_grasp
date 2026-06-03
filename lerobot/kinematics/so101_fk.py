import math

import numpy as np

from lerobot.kinematics.so101_model import ACTIVE_JOINT_NAMES


def forward_kinematics(model, joint_positions):
    _validate_joint_positions(model, joint_positions)

    transforms = {model.base_link: np.eye(4)}
    for joint in model.joints:
        if joint.parent not in transforms:
            continue

        parent_transform = transforms[joint.parent]
        joint_transform = origin_transform(joint.origin_xyz, joint.origin_rpy)
        if joint.name in ACTIVE_JOINT_NAMES:
            joint_transform = joint_transform @ axis_rotation_transform(
                joint.axis,
                float(joint_positions.get(joint.name, 0.0)),
            )

        transforms[joint.child] = parent_transform @ joint_transform

    return transforms


def end_effector_transform(model, joint_positions):
    transforms = forward_kinematics(model, joint_positions)
    return transforms[model.end_effector_link]


def origin_transform(xyz, rpy):
    transform = np.eye(4)
    transform[:3, :3] = rpy_matrix(*rpy)
    transform[:3, 3] = np.array(xyz, dtype=float)
    return transform


def axis_rotation_transform(axis, angle):
    transform = np.eye(4)
    transform[:3, :3] = axis_angle_matrix(axis, angle)
    return transform


def rpy_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rotation_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ]
    )
    rotation_y = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ]
    )
    rotation_z = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return rotation_z @ rotation_y @ rotation_x


def axis_angle_matrix(axis, angle):
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(3)
    x, y, z = axis / norm

    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [
                c + x * x * one_minus_c,
                x * y * one_minus_c - z * s,
                x * z * one_minus_c + y * s,
            ],
            [
                y * x * one_minus_c + z * s,
                c + y * y * one_minus_c,
                y * z * one_minus_c - x * s,
            ],
            [
                z * x * one_minus_c - y * s,
                z * y * one_minus_c + x * s,
                c + z * z * one_minus_c,
            ],
        ]
    )


def _validate_joint_positions(model, joint_positions):
    limits = model.joint_limits()
    for joint_name, value in joint_positions.items():
        if joint_name not in limits:
            raise ValueError(f"unknown joint: {joint_name}")

        lower, upper = limits[joint_name]
        value = float(value)
        if lower is not None and value < lower:
            raise ValueError(
                f"{joint_name} target {value} outside limits [{lower}, {upper}]"
            )
        if upper is not None and value > upper:
            raise ValueError(
                f"{joint_name} target {value} outside limits [{lower}, {upper}]"
            )
