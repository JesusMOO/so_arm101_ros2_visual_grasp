import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT_DIR / "data"
DEFAULT_CAMERA_CALIBRATION_PATH = None
DEFAULT_CAMERA_TO_BASE_TRANSFORM_PATH = None
DEFAULT_WORKSPACE_TO_BASE_PATH = None
DEFAULT_BLOCK_LOCATION_PATH = None
PARALLEL_EPSILON = 1e-9


@dataclass(frozen=True)
class CameraCalibration:
    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    distortion: np.ndarray


@dataclass(frozen=True)
class Transform:
    parent_frame: str
    child_frame: str
    translation_m: np.ndarray
    rotation_xyzw: np.ndarray


@dataclass(frozen=True)
class PlaneWorkspaceMapping:
    workspace_width_px: float
    workspace_height_px: float
    origin_base_m: np.ndarray
    x_axis_base_m: np.ndarray
    y_axis_base_m: np.ndarray


def load_camera_calibration(path=DEFAULT_CAMERA_CALIBRATION_PATH):
    path = path if path is not None else default_data_path("camera_calibration.json")
    payload = _read_json(path)
    return CameraCalibration(
        image_width=int(payload["image_width"]),
        image_height=int(payload["image_height"]),
        camera_matrix=np.array(payload["camera_matrix"], dtype=float),
        distortion=np.array(payload.get("distortion", []), dtype=float),
    )


def load_camera_to_base_transform(path=DEFAULT_CAMERA_TO_BASE_TRANSFORM_PATH):
    path = (
        path
        if path is not None
        else default_data_path("camera_to_base_transform.json")
    )
    payload = _read_json(path)
    return Transform(
        parent_frame=str(payload["parent_frame"]),
        child_frame=str(payload["child_frame"]),
        translation_m=_vector_from_payload(payload["translation_m"]),
        rotation_xyzw=_quaternion_from_payload(payload["rotation_xyzw"]),
    )


def load_plane_workspace_mapping(path=DEFAULT_WORKSPACE_TO_BASE_PATH):
    path = (
        path
        if path is not None
        else default_data_path("workspace_to_base_transform.json")
    )
    payload = _read_json(path)
    workspace_size = payload["workspace_size_px"]
    return PlaneWorkspaceMapping(
        workspace_width_px=float(workspace_size["width"]),
        workspace_height_px=float(workspace_size["height"]),
        origin_base_m=_vector_from_payload(payload["origin_base_m"]),
        x_axis_base_m=_vector_from_payload(payload["x_axis_base_m"]),
        y_axis_base_m=_vector_from_payload(payload["y_axis_base_m"]),
    )


def load_block_center(path=DEFAULT_BLOCK_LOCATION_PATH):
    path = path if path is not None else default_data_path("block_location.json")
    payload = _read_json(path)
    if not payload.get("found") or payload.get("center") is None:
        raise ValueError("no red block detection in block location file")
    center = payload["center"]
    return float(center["x"]), float(center["y"])


def pixel_to_camera_ray(calibration, u, v):
    fx = calibration.camera_matrix[0, 0]
    fy = calibration.camera_matrix[1, 1]
    cx = calibration.camera_matrix[0, 2]
    cy = calibration.camera_matrix[1, 2]
    if fx == 0.0 or fy == 0.0:
        raise ValueError("camera focal length must be non-zero")

    return np.array(
        [
            (float(u) - cx) / fx,
            (float(v) - cy) / fy,
            1.0,
        ],
        dtype=float,
    )


def camera_pixel_to_base_point_on_plane(
    calibration,
    camera_to_base,
    u,
    v,
    plane_z_m,
):
    ray_camera = pixel_to_camera_ray(calibration, u, v)
    rotation_base_camera = quaternion_xyzw_to_matrix(camera_to_base.rotation_xyzw)
    ray_base = rotation_base_camera @ ray_camera
    origin_base = camera_to_base.translation_m

    if abs(ray_base[2]) <= PARALLEL_EPSILON:
        raise ValueError("camera ray is parallel to the target plane")

    scale = (float(plane_z_m) - origin_base[2]) / ray_base[2]
    if scale < 0.0:
        raise ValueError("target plane is behind the camera ray")

    return origin_base + scale * ray_base


def workspace_point_to_base_point(mapping, x, y):
    x = float(x)
    y = float(y)
    if (
        x < 0.0
        or x > mapping.workspace_width_px
        or y < 0.0
        or y > mapping.workspace_height_px
    ):
        raise ValueError(
            f"workspace point ({x}, {y}) outside workspace "
            f"[0, {mapping.workspace_width_px}] x [0, {mapping.workspace_height_px}]"
        )

    x_ratio = x / mapping.workspace_width_px
    y_ratio = y / mapping.workspace_height_px
    return (
        mapping.origin_base_m
        + x_ratio * mapping.x_axis_base_m
        + y_ratio * mapping.y_axis_base_m
    )


def block_location_to_base_point(
    block_location_path=DEFAULT_BLOCK_LOCATION_PATH,
    workspace_mapping_path=DEFAULT_WORKSPACE_TO_BASE_PATH,
):
    x, y = load_block_center(block_location_path)
    mapping = load_plane_workspace_mapping(workspace_mapping_path)
    return workspace_point_to_base_point(mapping, x, y)


def default_data_path(filename):
    source_path = DATA_DIR / filename
    if source_path.exists():
        return source_path

    cwd_path = Path.cwd() / "data" / filename
    if cwd_path.exists():
        return cwd_path

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return source_path

    return Path(get_package_share_directory("lerobot")) / "data" / filename


def quaternion_xyzw_to_matrix(quaternion_xyzw):
    quaternion = np.array(quaternion_xyzw, dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        raise ValueError("quaternion must be non-zero")
    x, y, z, w = quaternion / norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


def _read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _vector_from_payload(payload):
    return np.array(
        [
            float(payload["x"]),
            float(payload["y"]),
            float(payload["z"]),
        ],
        dtype=float,
    )


def _quaternion_from_payload(payload):
    return np.array(
        [
            float(payload["x"]),
            float(payload["y"]),
            float(payload["z"]),
            float(payload["w"]),
        ],
        dtype=float,
    )
