from lerobot.so101.arm_backend import (
    ArmBackend,
    DEFAULT_ACC,
    DEFAULT_SPEED,
    validate_joint_targets,
)
from lerobot.so101.joint_mapping import JOINT_NAMES


class SimArmBackend(ArmBackend):
    def __init__(self, initial_positions=None):
        self.positions = {joint_name: 0.0 for joint_name in JOINT_NAMES}
        if initial_positions:
            self.positions.update(validate_joint_targets(initial_positions))

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
        return True

    def stop(self):
        return True
