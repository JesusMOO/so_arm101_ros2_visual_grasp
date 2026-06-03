from abc import ABC, abstractmethod

from lerobot.so101.joint_mapping import JOINT_CALIBRATIONS, JOINT_NAMES, SERVO_IDS


DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_SPEED = 600
DEFAULT_ACC = 20
OPEN_GRIPPER_RAD = 1.2
CLOSED_GRIPPER_RAD = -0.08
#安全检查抽象层，要角度先来这里检查是否越界，在调用底层
JOINT_LIMITS = {
    calibration.joint_name: (calibration.lower_rad, calibration.upper_rad)
    for calibration in JOINT_CALIBRATIONS.values()
}
JOINT_TO_SERVO = {
    calibration.joint_name: servo_id
    for servo_id, calibration in JOINT_CALIBRATIONS.items()
}


def validate_joint_targets(target):
    validated = {}
    for joint_name, radians in target.items():
        if joint_name not in JOINT_LIMITS:
            raise ValueError(f"unknown joint: {joint_name}")

        lower, upper = JOINT_LIMITS[joint_name]
        radians = float(radians)
        if radians < lower or radians > upper:
            raise ValueError(
                f"{joint_name} target {radians} outside limits [{lower}, {upper}]"
            )
        validated[joint_name] = radians
    return validated


class ArmBackend(ABC):
    @abstractmethod
    def get_joint_positions(self):
        pass

    @abstractmethod
    def move_joints(
        self,
        target,
        speed=DEFAULT_SPEED,
        acc=DEFAULT_ACC,
        wait=False,
    ):
        pass

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

    @abstractmethod
    def stop(self):
        pass

    def close(self):
        pass
