from dataclasses import dataclass

import numpy as np

from lerobot.kinematics.so101_fk import end_effector_transform
from lerobot.kinematics.so101_model import ACTIVE_JOINT_NAMES


DEFAULT_POSITION_TOLERANCE = 1e-4
DEFAULT_MAX_ITERATIONS = 120
DEFAULT_DAMPING = 1e-3
DEFAULT_STEP_LIMIT = 0.15
FINITE_DIFFERENCE_STEP = 1e-5


@dataclass(frozen=True)
class IKResult:
    success: bool
    joint_positions: dict
    position_error_norm: float
    iterations: int
    message: str


def solve_position_ik(
    model,
    target_position,
    initial_positions=None,
    locked_joints=None,
    position_tolerance=DEFAULT_POSITION_TOLERANCE,
    max_iterations=DEFAULT_MAX_ITERATIONS,
    damping=DEFAULT_DAMPING,
    step_limit=DEFAULT_STEP_LIMIT,
):
    target_position = _as_position_vector(target_position)
    joint_positions = _initial_joint_positions(model, initial_positions)
    locked_positions = _locked_joint_positions(model, locked_joints)
    joint_positions.update(locked_positions)
    joint_limits = model.joint_limits()
    inactive_joints = set(locked_positions) | {"gripper"}

    for iteration in range(max_iterations + 1):
        current_position = _end_effector_position(model, joint_positions)
        error = target_position - current_position
        error_norm = float(np.linalg.norm(error))
        if error_norm <= position_tolerance:
            return IKResult(
                success=True,
                joint_positions=dict(joint_positions),
                position_error_norm=error_norm,
                iterations=iteration,
                message="solved",
            )

        if iteration == max_iterations:
            break

        jacobian = _finite_difference_position_jacobian(
            model,
            joint_positions,
            current_position,
            inactive_joints,
        )
        delta = _damped_least_squares_step(jacobian, error, damping)
        delta = _limit_step(delta, step_limit)

        for joint_name, joint_delta in zip(ACTIVE_JOINT_NAMES, delta):
            if joint_name in inactive_joints:
                continue
            joint_positions[joint_name] = _clamp_to_limits(
                joint_positions[joint_name] + float(joint_delta),
                joint_limits[joint_name],
            )

    current_position = _end_effector_position(model, joint_positions)
    error_norm = float(np.linalg.norm(target_position - current_position))
    return IKResult(
        success=False,
        joint_positions=dict(joint_positions),
        position_error_norm=error_norm,
        iterations=max_iterations,
        message="failed to solve position IK",
    )


def _as_position_vector(value):
    vector = np.array(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError("target_position must be a 3 element vector")
    return vector


def _initial_joint_positions(model, initial_positions):
    joint_limits = model.joint_limits()
    positions = {joint_name: 0.0 for joint_name in ACTIVE_JOINT_NAMES}
    if initial_positions:
        for joint_name, value in initial_positions.items():
            if joint_name not in joint_limits:
                raise ValueError(f"unknown joint: {joint_name}")
            positions[joint_name] = float(value)

    _validate_limits(positions, joint_limits)
    return positions


def _locked_joint_positions(model, locked_joints):
    if not locked_joints:
        return {}

    joint_limits = model.joint_limits()
    positions = {}
    for joint_name, value in locked_joints.items():
        if joint_name not in joint_limits:
            raise ValueError(f"unknown joint: {joint_name}")
        positions[joint_name] = float(value)

    _validate_limits(positions, joint_limits)
    return positions


def _validate_limits(joint_positions, joint_limits):
    for joint_name, value in joint_positions.items():
        lower, upper = joint_limits[joint_name]
        if lower is not None and value < lower:
            raise ValueError(
                f"{joint_name} target {value} outside limits [{lower}, {upper}]"
            )
        if upper is not None and value > upper:
            raise ValueError(
                f"{joint_name} target {value} outside limits [{lower}, {upper}]"
            )


def _end_effector_position(model, joint_positions):
    return end_effector_transform(model, joint_positions)[:3, 3]


def _finite_difference_position_jacobian(
    model,
    joint_positions,
    current_position,
    inactive_joints,
):
    columns = []
    joint_limits = model.joint_limits()
    for joint_name in ACTIVE_JOINT_NAMES:
        if joint_name in inactive_joints:
            columns.append(np.zeros(3))
            continue

        perturbed = dict(joint_positions)
        perturbed[joint_name] = _clamp_to_limits(
            perturbed[joint_name] + FINITE_DIFFERENCE_STEP,
            joint_limits[joint_name],
        )
        moved_position = _end_effector_position(model, perturbed)
        columns.append((moved_position - current_position) / FINITE_DIFFERENCE_STEP)
    return np.column_stack(columns)


def _damped_least_squares_step(jacobian, error, damping):
    left = jacobian @ jacobian.T + (damping ** 2) * np.eye(3)
    return jacobian.T @ np.linalg.solve(left, error)


def _limit_step(delta, step_limit):
    norm = float(np.linalg.norm(delta))
    if norm <= step_limit:
        return delta
    return delta * (step_limit / norm)


def _clamp_to_limits(value, limits):
    lower, upper = limits
    if lower is not None:
        value = max(lower, value)
    if upper is not None:
        value = min(upper, value)
    return value
