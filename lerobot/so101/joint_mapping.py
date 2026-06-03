import math
from dataclasses import dataclass

#舵机 raw 值和 URDF 关节角之间的转换
SERVO_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
RAW_MIN = 0
RAW_MAX = 4095
RAW_CENTER = 2048
RAW_TICKS_PER_RAD = 4096 / (2 * math.pi)


@dataclass(frozen=True)
class JointCalibration:
    servo_id: int
    joint_name: str
    lower_rad: float
    upper_rad: float
    raw_center: int = RAW_CENTER
    direction: int = 1
    rad_offset: float = 0.0


JOINT_CALIBRATIONS = {
    1: JointCalibration(1, "shoulder_pan", -1.91986, 1.91986),
    2: JointCalibration(2, "shoulder_lift", -1.74533, 1.74533,rad_offset=-1.745),
    3: JointCalibration(3, "elbow_flex", -1.69, 1.69,rad_offset=1.55),
    4: JointCalibration(4, "wrist_flex", -1.65806, 1.65806,rad_offset=1.28),
    5: JointCalibration(5, "wrist_roll", -2.74385, 2.84121,rad_offset=1.59),
    6: JointCalibration(6, "gripper", -0.174533, 1.74533,rad_offset=-0.1745),
}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def raw_to_rad(servo_id, raw):
    calibration = JOINT_CALIBRATIONS[servo_id]
    raw = clamp(int(raw), RAW_MIN, RAW_MAX)
    radians = (
        ((raw - calibration.raw_center) / RAW_TICKS_PER_RAD)
        * calibration.direction
        + calibration.rad_offset
    )
    return clamp(radians, calibration.lower_rad, calibration.upper_rad)


def rad_to_raw(servo_id, radians):
    calibration = JOINT_CALIBRATIONS[servo_id]
    radians = clamp(float(radians), calibration.lower_rad, calibration.upper_rad)
    raw = (
        ((radians - calibration.rad_offset) / calibration.direction)
        * RAW_TICKS_PER_RAD
        + calibration.raw_center
    )
    return int(round(clamp(raw, RAW_MIN, RAW_MAX)))


def raw_positions_to_joint_positions(raw_positions):
    return {
        JOINT_CALIBRATIONS[servo_id].joint_name: raw_to_rad(
            servo_id,
            raw_positions[servo_id],
        )
        for servo_id in SERVO_IDS
    }
