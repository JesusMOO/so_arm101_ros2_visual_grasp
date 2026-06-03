import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from lerobot.grasp.red_block_grasp_controller import RedBlockGraspController
from lerobot.so101.arm_backend import DEFAULT_ACC, DEFAULT_SERIAL_PORT, DEFAULT_SPEED
from lerobot.so101.real_arm_backend import RealArmBackend
from lerobot.so101.sim_arm_backend import SimArmBackend
from lerobot.vision.camera_geometry import block_location_to_base_point


class RedBlockGraspControllerNode(Node):
    def __init__(self):
        super().__init__("red_block_grasp_controller")

        self.declare_parameter("backend", "sim")
        self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT)
        self.declare_parameter("speed", DEFAULT_SPEED)
        self.declare_parameter("acc", DEFAULT_ACC)
        self.declare_parameter("wait", True)
        self.declare_parameter("block_location_path", "")
        self.declare_parameter("workspace_mapping_path", "")
        self.declare_parameter("fixed_grasp_wrist_roll_rad", 1.59)

        self.backend_name = str(self.get_parameter("backend").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.speed = int(self.get_parameter("speed").value)
        self.acc = int(self.get_parameter("acc").value)
        self.wait = bool(self.get_parameter("wait").value)
        self.block_location_path = str(
            self.get_parameter("block_location_path").value
        )
        self.workspace_mapping_path = str(
            self.get_parameter("workspace_mapping_path").value
        )
        self.fixed_grasp_wrist_roll_rad = float(
            self.get_parameter("fixed_grasp_wrist_roll_rad").value
        )

        self.backend = self._create_backend()
        self.state_publisher = self.create_publisher(
            String,
            "/so101/grasp_state",
            10,
        )
        self.service = self.create_service(
            Trigger,
            "/so101/pick_red_block",
            self.handle_pick_request,
        )
        self.controller = RedBlockGraspController(
            backend=self.backend,
            block_point_provider=self._block_point_provider,
            state_callback=self.publish_state,
            speed=self.speed,
            acc=self.acc,
            wait=self.wait,
            fixed_grasp_wrist_roll_rad=self.fixed_grasp_wrist_roll_rad,
        )
        self.publish_state("IDLE")

    def _create_backend(self):
        if self.backend_name == "sim":
            self.get_logger().info("using sim arm backend")
            return SimArmBackend()
        if self.backend_name == "real":
            self.get_logger().info(
                f"using real arm backend on serial port {self.serial_port}"
            )
            return RealArmBackend(serial_port=self.serial_port)
        raise ValueError("backend must be 'sim' or 'real'")

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)

    def _block_point_provider(self):
        block_location_path = self.block_location_path or None
        workspace_mapping_path = self.workspace_mapping_path or None
        return block_location_to_base_point(
            block_location_path=block_location_path,
            workspace_mapping_path=workspace_mapping_path,
        )

    def handle_pick_request(self, request, response):
        del request
        result = self.controller.execute_pick()
        response.success = result.success
        response.message = result.message
        return response

    def destroy_node(self):
        if hasattr(self.backend, "close"):
            self.backend.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RedBlockGraspControllerNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
