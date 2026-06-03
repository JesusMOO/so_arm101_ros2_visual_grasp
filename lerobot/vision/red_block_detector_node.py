from pathlib import Path

import cv2
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker  #用于rviz的显示

from lerobot.vision.detect_red_block import (
    CAMERA_INDEX,
    MIN_RED_AREA,
    detect_largest_red_block,
    draw_detection,
    load_plane_transform,
    save_block_location,
)


DEFAULT_FRAME_ID = "workspace"
DEFAULT_TIMER_PERIOD = 0.033
DEFAULT_SHOW_DEBUG = True
DEFAULT_DATA_DIR = Path("/home/spenta/lerobot/data")
DEFAULT_PLANE_TRANSFORM_PATH = DEFAULT_DATA_DIR / "plane_transform.json"
DEFAULT_BLOCK_LOCATION_PATH = DEFAULT_DATA_DIR / "block_location.json"


def build_detection_messages(detection, frame_id=DEFAULT_FRAME_ID, stamp=None):
    found_msg = Bool()
    found_msg.data = detection is not None

    marker_msg = Marker()
    marker_msg.header.frame_id = frame_id
    if stamp is not None:
        marker_msg.header.stamp = stamp
    marker_msg.ns = "red_block"
    marker_msg.id = 0

    if detection is None:
        marker_msg.action = Marker.DELETE
        return found_msg, None, marker_msg

    x, y = detection["center"]
    bx, by, bw, bh = detection["bbox"]

    center_msg = PointStamped()
    center_msg.header.frame_id = frame_id
    if stamp is not None:
        center_msg.header.stamp = stamp
    center_msg.point.x = float(x)
    center_msg.point.y = float(y)
    center_msg.point.z = 0.0

    marker_msg.type = Marker.CUBE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = float(x)
    marker_msg.pose.position.y = float(y)
    marker_msg.pose.position.z = 0.0
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = float(bw)
    marker_msg.scale.y = float(bh)
    marker_msg.scale.z = 1.0
    marker_msg.color.r = 1.0
    marker_msg.color.g = 0.0
    marker_msg.color.b = 0.0
    marker_msg.color.a = 0.75

    return found_msg, center_msg, marker_msg


class RedBlockDetectorNode(Node):
    def __init__(self):
        super().__init__("red_block_detector")

        self.declare_parameter("camera_index", CAMERA_INDEX)
        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("min_red_area", MIN_RED_AREA)
        self.declare_parameter(
            "plane_transform_path",
            str(DEFAULT_PLANE_TRANSFORM_PATH),
        )
        self.declare_parameter(
            "block_location_path",
            str(DEFAULT_BLOCK_LOCATION_PATH),
        )
        self.declare_parameter("timer_period", DEFAULT_TIMER_PERIOD)
        self.declare_parameter("show_debug", DEFAULT_SHOW_DEBUG)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.min_red_area = float(self.get_parameter("min_red_area").value)
        self.plane_transform_path = Path(
            str(self.get_parameter("plane_transform_path").value)
        )
        self.block_location_path = Path(
            str(self.get_parameter("block_location_path").value)
        )
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)

        self.transform_config = load_plane_transform(self.plane_transform_path)
        self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            self.get_logger().error(
                f"camera open failed: index={self.camera_index}"
            )

        self.found_pub = self.create_publisher(
            Bool,
            "/red_block/found",
            10,
        )
        self.center_pub = self.create_publisher(
            PointStamped,
            "/red_block/center",
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            "/red_block/debug_marker",
            10,
        )
        self.timer = self.create_timer(self.timer_period, self.process_frame)

    def process_frame(self):
        if not self.camera.isOpened():
            return

        ok, frame = self.camera.read()
        if not ok:
            self.get_logger().warning("camera read failed")
            return

        warped = cv2.warpPerspective(
            frame,
            self.transform_config["transform_matrix"],
            (
                self.transform_config["target_width"],
                self.transform_config["target_height"],
            ),
        )
        detection = detect_largest_red_block(
            warped,
            min_area=self.min_red_area,
        )
        save_block_location(self.block_location_path, detection)
        self.publish_detection(detection)

        if self.show_debug:
            cv2.imshow("original_frame", frame)
            cv2.imshow("red_block_detection", draw_detection(warped, detection))
            cv2.waitKey(1)

    def publish_detection(self, detection):
        stamp = self.get_clock().now().to_msg()
        found_msg, center_msg, marker_msg = build_detection_messages(
            detection,
            frame_id=self.frame_id,
            stamp=stamp,
        )

        self.found_pub.publish(found_msg)
        if center_msg is not None:
            self.center_pub.publish(center_msg)
        self.marker_pub.publish(marker_msg)

    def destroy_node(self):
        if hasattr(self, "camera"):
            self.camera.release()
        if getattr(self, "show_debug", False):
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RedBlockDetectorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
