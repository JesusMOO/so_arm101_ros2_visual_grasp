from lerobot.so101.arm_backend import (
    ArmBackend,
    DEFAULT_ACC,
    DEFAULT_SERIAL_PORT,
    DEFAULT_SPEED,
    JOINT_TO_SERVO,
    validate_joint_targets,
)
from lerobot.so101.joint_mapping import (
    JOINT_NAMES,
    SERVO_IDS,
    rad_to_raw,
    raw_positions_to_joint_positions,
)
from lerobot.so101.servo_bus import ServoBus


class RealArmBackend(ArmBackend):
    def __init__(self, serial_port=DEFAULT_SERIAL_PORT, bus=None):
        self.bus = bus if bus is not None else ServoBus(serial_port)

    def get_joint_positions(self):
        raw_positions = {}
        for servo_id in SERVO_IDS:
            raw = self.bus.read_position(servo_id)
            if raw is None:
                raise RuntimeError(f"failed to read servo {servo_id} position")
            raw_positions[servo_id] = raw
        return raw_positions_to_joint_positions(raw_positions)

    def move_joints(
        self,
        target,
        speed=DEFAULT_SPEED,
        acc=DEFAULT_ACC,
        wait=False,
    ):
        validated = validate_joint_targets(target)
        for joint_name in JOINT_NAMES:
            if joint_name not in validated:
                continue

            servo_id = JOINT_TO_SERVO[joint_name]
            raw = rad_to_raw(servo_id, validated[joint_name])
            result = self.bus.move_to(
                servo_id,
                raw,
                speed=speed,
                acc=acc,
                wait=wait,
            )
            if result is None:
                raise RuntimeError(f"failed to move servo {servo_id}")
        return True

    def stop(self):
        for servo_id in SERVO_IDS:
            if hasattr(self.bus, "stop"):
                self.bus.stop(servo_id)
        return True

    def close(self):
        self.bus.close()
