import json
from pathlib import Path

import cv2


ROOT_DIR = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT_DIR / "data"
WINDOW_NAME = "workspace_calibration"
SAVE_PATH = DATA_DIR / "workspace_points.json"
CAMERA_INDEX = 4


class ManualWorkspaceCalibrator:
    def __init__(self, max_points=4):
        self.max_points = max_points
        self.points = []

    def add_point(self, x, y):
        if len(self.points) < self.max_points:
            self.points.append((int(x), int(y)))

    def reset(self):
        self.points.clear()

    def is_complete(self):
        return len(self.points) == self.max_points

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.add_point(x, y)


def save_workspace_points(path, points):
    payload = {
        "points": [{"x": int(x), "y": int(y)} for x, y in points],
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_workspace_points(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return [(int(item["x"]), int(item["y"])) for item in payload["points"]]


def workspace_segments(points):
    if len(points) < 2:
        return []

    segments = []
    for index in range(len(points) - 1):
        segments.append((points[index], points[index + 1]))

    if len(points) == 4:
        segments.append((points[-1], points[0]))

    return segments


def draw_workspace(frame, points):
    preview = frame.copy()
    cv2.putText(
        preview,
        "click 4 points | r reset | s save | q quit",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )

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


def read_key():
    key = cv2.waitKeyEx(30)
    if key != -1:
        return key & 0xFF
    return -1


def main():
    cam = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cam.isOpened():
        print("camera open failed")
        return

    calibrator = ManualWorkspaceCalibrator()
    if SAVE_PATH.exists():
        calibrator.points = load_workspace_points(SAVE_PATH)

    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, calibrator.on_mouse)

    print("Left click points in this order: top-left, top-right, bottom-right, bottom-left")
    print("Press r to reset, s to save, q to quit")

    try:
        while True:
            ok, frame = cam.read()
            if not ok:
                print("camera read failed")
                break

            preview = draw_workspace(frame, calibrator.points)
            cv2.imshow(WINDOW_NAME, preview)

            key = read_key()
            if key == ord("r"):
                calibrator.reset()
            elif key == ord("s"):
                if calibrator.is_complete():
                    save_workspace_points(SAVE_PATH, calibrator.points)
                    print(f"saved workspace points to {SAVE_PATH}")
                else:
                    print("need exactly 4 points before saving")
            elif key == ord("q"):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
