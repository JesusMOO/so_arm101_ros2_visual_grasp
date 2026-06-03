import json
from pathlib import Path

import cv2
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT_DIR / "data"
WORKSPACE_POINTS_PATH = DATA_DIR / "workspace_points.json"
PLANE_TRANSFORM_PATH = DATA_DIR / "plane_transform.json"
ORIGINAL_WINDOW_NAME = "original_workspace"
WARPED_WINDOW_NAME = "warped_workspace"
TARGET_WIDTH = 300
TARGET_HEIGHT = 200
CAMERA_INDEX = 4


def load_workspace_points(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return [(int(item["x"]), int(item["y"])) for item in payload["points"]]


def destination_points(width, height):
    return [(0, 0), (width, 0), (width, height), (0, height)]


def points_to_array(points):
    return np.array(points, dtype=np.float32)


def build_transform_payload(source_points, destination_points, transform_matrix, inverse_transform_matrix, width, height):
    return {
        "source_points": [{"x": int(x), "y": int(y)} for x, y in source_points],
        "destination_points": [{"x": int(x), "y": int(y)} for x, y in destination_points],
        "target_width": int(width),
        "target_height": int(height),
        "transform_matrix": transform_matrix.tolist(),
        "inverse_transform_matrix": inverse_transform_matrix.tolist(),
    }


def save_plane_transform(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def workspace_segments(points):
    if len(points) < 2:
        return []

    segments = []
    for index in range(len(points) - 1):
        segments.append((points[index], points[index + 1]))

    if len(points) == 4:
        segments.append((points[-1], points[0]))

    return segments


def draw_workspace_outline(frame, points):
    preview = frame.copy()

    for index, (x, y) in enumerate(points, start=1):
        cv2.circle(preview, (x, y), 5, (0, 255, 0), -1)
        cv2.putText(
            preview,
            f"P{index} ({x}, {y})",
            (x + 8, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

    for start, end in workspace_segments(points):
        cv2.line(preview, start, end, (255, 0, 0), 2)

    return preview


def compute_transform(source_points, width, height):
    dst_points = destination_points(width, height)
    src_array = points_to_array(source_points)
    dst_array = points_to_array(dst_points)
    transform_matrix = cv2.getPerspectiveTransform(src_array, dst_array)
    inverse_transform_matrix = cv2.getPerspectiveTransform(dst_array, src_array)
    return dst_points, transform_matrix, inverse_transform_matrix


def main():
    source_points = load_workspace_points(WORKSPACE_POINTS_PATH)
    if len(source_points) != 4:
        print("workspace_points.json must contain exactly 4 points")
        return

    destination, transform_matrix, inverse_transform_matrix = compute_transform(
        source_points,
        TARGET_WIDTH,
        TARGET_HEIGHT,
    )

    payload = build_transform_payload(
        source_points=source_points,
        destination_points=destination,
        transform_matrix=transform_matrix,
        inverse_transform_matrix=inverse_transform_matrix,
        width=TARGET_WIDTH,
        height=TARGET_HEIGHT,
    )
    save_plane_transform(PLANE_TRANSFORM_PATH, payload)
    print(f"saved plane transform to {PLANE_TRANSFORM_PATH}")

    cam = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cam.isOpened():
        print("camera open failed")
        return

    while True:
        ok, frame = cam.read()
        if not ok:
            print("camera read failed")
            break

        original_preview = draw_workspace_outline(frame, source_points)
        warped = cv2.warpPerspective(frame, transform_matrix, (TARGET_WIDTH, TARGET_HEIGHT))

        cv2.imshow(ORIGINAL_WINDOW_NAME, original_preview)
        cv2.imshow(WARPED_WINDOW_NAME, warped)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
