from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

from lerobot.so101.joint_mapping import (
    JOINT_NAMES,
    SERVO_IDS,
    raw_positions_to_joint_positions,
)
from lerobot.so101.servo_bus import ServoBus


DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_PUBLISH_RATE = 20.0

#负责读取真实舵机位置并发布 /joint_states
def build_joint_state_message(raw_positions, stamp=None):
    joint_positions = raw_positions_to_joint_positions(raw_positions)

    msg = JointState()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.name = list(JOINT_NAMES)
    msg.position = [joint_positions[name] for name in JOINT_NAMES]
    return msg


class SO101JointStatePublisherNode(Node):
    def __init__(self):
        super().__init__("so101_joint_state_publisher")

        self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT)
        self.declare_parameter("publish_rate", DEFAULT_PUBLISH_RATE)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.bus = None

        try:
            self.bus = ServoBus(self.serial_port)
            self.get_logger().info(f"opened SO101 servo bus: {self.serial_port}")
        except Exception as exc:
            self.get_logger().error(
                f"failed to open SO101 servo bus {self.serial_port}: {exc}"
            )

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

    def read_raw_positions(self):
        if self.bus is None:
            return None

        readings = {}
        for servo_id in SERVO_IDS:
            raw = self.bus.read_position(servo_id)
            if raw is None:
                self.get_logger().warning(
                    f"failed to read servo {servo_id} position"
                )
                return None
            readings[servo_id] = raw
        return readings

    def publish(self):
        readings = self.read_raw_positions()
        if readings is None:
            return

        msg = build_joint_state_message(
            readings,
            stamp=self.get_clock().now().to_msg(),
        )
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.bus is not None:
            self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SO101JointStatePublisherNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
