from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    calibration = LaunchConfiguration("calibration").perform(context)
    camera_index = LaunchConfiguration("camera_index")
    serial_port = LaunchConfiguration("serial_port")
    joint_publish_rate = LaunchConfiguration("joint_publish_rate")
    speed = LaunchConfiguration("speed")
    acc = LaunchConfiguration("acc")
    wait = LaunchConfiguration("wait")
    show_debug = LaunchConfiguration("show_debug")
    fixed_grasp_wrist_roll_rad = LaunchConfiguration("fixed_grasp_wrist_roll_rad")
    start_joint_state_publisher = LaunchConfiguration("start_joint_state_publisher")
    auto_pick = LaunchConfiguration("auto_pick")
    auto_pick_delay = LaunchConfiguration("auto_pick_delay")

    package_share = Path(get_package_share_directory("lerobot"))
    repo_root = Path("/home/spenta/lerobot")
    data_dir = repo_root / "data"
    urdf_path = package_share / "urdf" / "so101" / f"so101_{calibration}_calib.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")

    plane_transform_path = data_dir / "plane_transform.json"
    block_location_path = data_dir / "block_location.json"
    workspace_mapping_path = data_dir / "workspace_to_base_transform.json"

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="lerobot",
            executable="so101_joint_state_publisher_node",
            name="so101_joint_state_publisher",
            parameters=[
                {
                    "serial_port": serial_port,
                    "publish_rate": joint_publish_rate,
                }
            ],
            condition=IfCondition(start_joint_state_publisher),
            output="screen",
        ),
        Node(
            package="lerobot",
            executable="red_block_detector_node",
            name="red_block_detector",
            parameters=[
                {
                    "camera_index": camera_index,
                    "plane_transform_path": str(plane_transform_path),
                    "block_location_path": str(block_location_path),
                    "show_debug": show_debug,
                }
            ],
            output="screen",
        ),
        Node(
            package="lerobot",
            executable="red_block_grasp_controller_node",
            name="red_block_grasp_controller",
            parameters=[
                {
                    "backend": "real",
                    "serial_port": serial_port,
                    "speed": speed,
                    "acc": acc,
                    "wait": wait,
                    "fixed_grasp_wrist_roll_rad": fixed_grasp_wrist_roll_rad,
                    "block_location_path": str(block_location_path),
                    "workspace_mapping_path": str(workspace_mapping_path),
                }
            ],
            output="screen",
        ),
        TimerAction(
            period=auto_pick_delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/so101/pick_red_block",
                        "std_srvs/srv/Trigger",
                        "{}",
                    ],
                    output="screen",
                )
            ],
            condition=IfCondition(auto_pick),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration",
                default_value="new",
                choices=["new", "old"],
                description="SO101 calibration model to display.",
            ),
            DeclareLaunchArgument(
                "camera_index",
                default_value="4",
                description="OpenCV camera index for the USB camera.",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Serial port for SO101 ST3215 servos.",
            ),
            DeclareLaunchArgument(
                "joint_publish_rate",
                default_value="20.0",
                description="Real joint state publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "speed",
                default_value="240",
                description="Low servo speed for real grasping.",
            ),
            DeclareLaunchArgument(
                "acc",
                default_value="8",
                description="Low servo acceleration for real grasping.",
            ),
            DeclareLaunchArgument(
                "wait",
                default_value="true",
                choices=["true", "false"],
                description="Wait for each servo command before continuing.",
            ),
            DeclareLaunchArgument(
                "show_debug",
                default_value="true",
                choices=["true", "false"],
                description="Show OpenCV debug windows.",
            ),
            DeclareLaunchArgument(
                "fixed_grasp_wrist_roll_rad",
                default_value="1.59",
                description="Fixed wrist_roll angle used for IK grasp waypoints.",
            ),
            DeclareLaunchArgument(
                "start_joint_state_publisher",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Start the real /joint_states publisher. Keep false while the "
                    "real grasp controller owns the servo serial port."
                ),
            ),
            DeclareLaunchArgument(
                "auto_pick",
                default_value="true",
                choices=["true", "false"],
                description="Automatically call /so101/pick_red_block after delay.",
            ),
            DeclareLaunchArgument(
                "auto_pick_delay",
                default_value="10.0",
                description="Delay in seconds before auto_pick service call.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
