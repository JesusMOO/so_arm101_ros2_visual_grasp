from lerobot.so101.joint_mapping import JOINT_CALIBRATIONS


HOME_JOINT_POSITIONS = {
    calibration.joint_name: calibration.rad_offset
    for calibration in JOINT_CALIBRATIONS.values()
}
READY_JOINT_POSITIONS = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
