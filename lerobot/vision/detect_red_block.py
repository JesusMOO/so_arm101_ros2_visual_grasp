import json
from pathlib import Path

import cv2
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT_DIR / "data"
PLANE_TRANSFORM_PATH = DATA_DIR / "plane_transform.json"
BLOCK_LOCATION_PATH = DATA_DIR / "block_location.json"
ORIGINAL_WINDOW_NAME = "original_frame"
WARPED_WINDOW_NAME = "red_block_detection"
CAMERA_INDEX = 4
MIN_RED_AREA = 100


def load_plane_transform(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return {
        "target_width": int(payload["target_width"]),
        "target_height": int(payload["target_height"]),
        "transform_matrix": np.array(payload["transform_matrix"], dtype=np.float32),
        "inverse_transform_matrix": np.array(payload["inverse_transform_matrix"], dtype=np.float32),
    }


def red_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red_1 = np.array([0, 100, 80], dtype=np.uint8)
    upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red_2 = np.array([170, 100, 80], dtype=np.uint8)
    upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)

    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_largest_red_block(frame, min_area=MIN_RED_AREA):
    mask = red_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    center = (x + w // 2, y + h // 2)
    return {
        "center": center,
        "bbox": (x, y, w, h),
        "area": area,
        "mask": mask,
    }


def save_block_location(path, detection):
    if detection is None:
        payload = {
            "found": False,
            "center": None,
            "bbox": None,
            "area": 0,
        }
    else:
        x, y = detection["center"]
        bx, by, bw, bh = detection["bbox"]
        payload = {
            "found": True,
            "center": {"x": int(x), "y": int(y)},
            "bbox": {"x": int(bx), "y": int(by), "w": int(bw), "h": int(bh)},
            "area": float(detection["area"]),
        }

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def draw_detection(frame, detection):
    preview = frame.copy()
    if detection is None:
        cv2.putText(preview, "not found", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return preview

    x, y, w, h = detection["bbox"]
    cx, cy = detection["center"]
    area = detection["area"]
    cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(preview, (cx, cy), 4, (255, 0, 0), -1)
    cv2.putText(preview, f"center=({cx}, {cy})", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(preview, f"bbox=({x}, {y}, {w}, {h})", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(preview, f"area={area:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return preview


def main():
    transform_config = load_plane_transform(PLANE_TRANSFORM_PATH)
    cam = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cam.isOpened():
        print("camera open failed")
        return

    while True:
        ok, frame = cam.read()
        if not ok:
            print("camera read failed")
            break

        warped = cv2.warpPerspective(
            frame,
            transform_config["transform_matrix"],
            (transform_config["target_width"], transform_config["target_height"]),
        )
        detection = detect_largest_red_block(warped)
        save_block_location(BLOCK_LOCATION_PATH, detection)

        detection_view = draw_detection(warped, detection)
        cv2.imshow(ORIGINAL_WINDOW_NAME, frame)
        cv2.imshow(WARPED_WINDOW_NAME, detection_view)

        if detection is None:
            print("not found")
        else:
            center = detection["center"]
            bbox = detection["bbox"]
            area = detection["area"]
            print(f"center={center}, bbox={bbox}, area={area:.1f}")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
