from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from lerobot.grasp.red_block_grasp_controller import RedBlockGraspController
from lerobot.grasp.grasp_joint_targets import (
    HOME_JOINT_POSITIONS,
)
from lerobot.so101.arm_backend import (
    CLOSED_GRIPPER_RAD,
    DEFAULT_ACC,
    DEFAULT_SPEED,
    OPEN_GRIPPER_RAD,
    validate_joint_targets,
)
from lerobot.so101.joint_mapping import JOINT_NAMES


DEFAULT_PUBLISH_RATE = 30.0
DEFAULT_STEP_DURATION = 1.5


class RecordingSimBackend:
    def __init__(self, initial_positions=None):
        self.positions = {joint_name: 0.0 for joint_name in JOINT_NAMES}
        if initial_positions:
            self.positions.update(validate_joint_targets(initial_positions))
        self.waypoints = [dict(self.positions)]

    def get_joint_positions(self):
        return dict(self.positions)

    def move_joints(
        self,
        target,
        speed=DEFAULT_SPEED,
        acc=DEFAULT_ACC,
        wait=False,
    ):
        del speed, acc, wait
        self.positions.update(validate_joint_targets(target))
        self.waypoints.append(dict(self.positions))
        return True

    def open_gripper(self, speed=DEFAULT_SPEED, acc=DEFAULT_ACC, wait=False):
        return self.move_joints(
            {"gripper": OPEN_GRIPPER_RAD},
            speed=speed,
            acc=acc,
            wait=wait,
        )

    def close_gripper(self, speed=DEFAULT_SPEED, acc=DEFAULT_ACC, wait=False):
        return self.move_joints(
            {"gripper": CLOSED_GRIPPER_RAD},
            speed=speed,
            acc=acc,
            wait=wait,
        )

    def stop(self):
        return True

    def close(self):
        pass


def interpolate_joint_positions(start, end, alpha):
    alpha = max(0.0, min(1.0, float(alpha)))
    return {
        joint_name: float(start[joint_name])
        + (float(end[joint_name]) - float(start[joint_name])) * alpha
        for joint_name in JOINT_NAMES
    }


def build_joint_state_message(joint_positions, stamp=None):
    msg = JointState()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.name = list(JOINT_NAMES)
    msg.position = [float(joint_positions[joint_name]) for joint_name in JOINT_NAMES]
    return msg


class SO101SimGraspDemoNode(Node):
    def __init__(self):
        super().__init__("so101_sim_grasp_demo")

        self.declare_parameter("publish_rate", DEFAULT_PUBLISH_RATE)
        self.declare_parameter("step_duration", DEFAULT_STEP_DURATION)
        self.declare_parameter("speed", DEFAULT_SPEED)
        self.declare_parameter("acc", DEFAULT_ACC)
        self.declare_parameter("fixed_grasp_wrist_roll_rad", 0.0)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.step_duration = float(self.get_parameter("step_duration").value)
        self.speed = int(self.get_parameter("speed").value)
        self.acc = int(self.get_parameter("acc").value)
        self.fixed_grasp_wrist_roll_rad = float(
            self.get_parameter("fixed_grasp_wrist_roll_rad").value
        )
        if self.step_duration <= 0.0:
            raise ValueError("step_duration must be positive")

        self.joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            "/so101/grasp_state",
            10,
        )

        self.backend = RecordingSimBackend(initial_positions=HOME_JOINT_POSITIONS)
        self.controller = RedBlockGraspController(
            backend=self.backend,
            state_callback=self.publish_state,
            speed=self.speed,
            acc=self.acc,
            wait=False,
            fixed_grasp_wrist_roll_rad=self.fixed_grasp_wrist_roll_rad,
        )
        result = self.controller.execute_pick()
        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().error(result.message)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)

    def current_joint_positions(self):
        if len(self.backend.waypoints) == 1:
            return dict(self.backend.waypoints[0])

        elapsed = (
            self.get_clock().now().nanoseconds - self.start_time.nanoseconds
        ) / 1e9
        segment = int(elapsed / self.step_duration)
        if segment >= len(self.backend.waypoints) - 1:
            return dict(self.backend.waypoints[-1])

        alpha = (elapsed - segment * self.step_duration) / self.step_duration
        return interpolate_joint_positions(
            self.backend.waypoints[segment],
            self.backend.waypoints[segment + 1],
            alpha,
        )

    def publish(self):
        msg = build_joint_state_message(
            self.current_joint_positions(),
            stamp=self.get_clock().now().to_msg(),
        )
        self.joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SO101SimGraspDemoNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
