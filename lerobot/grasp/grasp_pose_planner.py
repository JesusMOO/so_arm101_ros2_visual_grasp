from dataclasses import dataclass, field

import numpy as np


DEFAULT_BLOCK_HEIGHT_M = 0.01
DEFAULT_GRASP_CLEARANCE_M = 0.0
DEFAULT_PRE_GRASP_HEIGHT_M = 0.02
DEFAULT_LIFT_HEIGHT_M = 0.12
DEFAULT_PLACE_POSITION_M = np.array([0.2, -0.15, 0.08], dtype=float)


@dataclass(frozen=True)
class GraspPoseConfig:
    block_height_m: float = DEFAULT_BLOCK_HEIGHT_M
    grasp_clearance_m: float = DEFAULT_GRASP_CLEARANCE_M
    pre_grasp_height_m: float = DEFAULT_PRE_GRASP_HEIGHT_M
    lift_height_m: float = DEFAULT_LIFT_HEIGHT_M
    place_position_m: np.ndarray = field(
        default_factory=lambda: DEFAULT_PLACE_POSITION_M.copy()
    )

    def __post_init__(self):
        for name in [
            "block_height_m",
            "grasp_clearance_m",
            "pre_grasp_height_m",
            "lift_height_m",
        ]:
            if float(getattr(self, name)) < 0.0:
                raise ValueError(f"{name} must be non-negative")

        place_position = _as_vector3(self.place_position_m, "place_position_m")
        object.__setattr__(self, "place_position_m", place_position)


@dataclass(frozen=True)
class GraspPosePlan:
    pre_grasp: np.ndarray
    grasp: np.ndarray
    lift: np.ndarray
    place: np.ndarray


def build_grasp_plan(block_base_point, config=None):
    config = config if config is not None else GraspPoseConfig()
    block_base_point = _as_vector3(block_base_point, "block_base_point")
    orientation = default_grasp_orientation()

    grasp_z = (
        block_base_point[2]
        + config.block_height_m / 2.0
        + config.grasp_clearance_m
    )
    pre_grasp_z = block_base_point[2] + config.block_height_m + config.pre_grasp_height_m
    lift_z = grasp_z + config.lift_height_m

    grasp_position = np.array(
        [block_base_point[0], block_base_point[1], grasp_z],
        dtype=float,
    )
    pre_grasp_position = np.array(
        [block_base_point[0], block_base_point[1], pre_grasp_z],
        dtype=float,
    )
    lift_position = np.array(
        [block_base_point[0], block_base_point[1], lift_z],
        dtype=float,
    )

    return GraspPosePlan(
        pre_grasp=make_pose(pre_grasp_position, orientation),
        grasp=make_pose(grasp_position, orientation),
        lift=make_pose(lift_position, orientation),
        place=make_pose(config.place_position_m, orientation),
    )


def make_pose(position, rotation):
    position = _as_vector3(position, "position")
    rotation = np.array(rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("rotation must be a 3x3 matrix")

    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = position
    return pose


def default_grasp_orientation():
    return np.eye(3)


def _as_vector3(value, name):
    vector = np.array(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} must be a 3 element vector")
    return vector
