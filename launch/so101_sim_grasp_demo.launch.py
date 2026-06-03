from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    calibration = LaunchConfiguration("calibration").perform(context)
    camera_index = LaunchConfiguration("camera_index")
    publish_rate = LaunchConfiguration("publish_rate")
    step_duration = LaunchConfiguration("step_duration")
    sim_start_delay = LaunchConfiguration("sim_start_delay")
    show_debug = LaunchConfiguration("show_debug")
    fixed_grasp_wrist_roll_rad = LaunchConfiguration("fixed_grasp_wrist_roll_rad")
    start_rviz = LaunchConfiguration("start_rviz")

    package_share = Path(get_package_share_directory("lerobot"))
    repo_root = Path("/home/spenta/lerobot")
    data_dir = repo_root / "data"
    urdf_path = package_share / "urdf" / "so101" / f"so101_{calibration}_calib.urdf"
    rviz_config_path = package_share / "rviz" / "so101.rviz"
    robot_description = urdf_path.read_text(encoding="utf-8")

    plane_transform_path = data_dir / "plane_transform.json"
    block_location_path = data_dir / "block_location.json"

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
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
        TimerAction(
            period=sim_start_delay,
            actions=[
                Node(
                    package="lerobot",
                    executable="so101_sim_grasp_demo_node",
                    name="so101_sim_grasp_demo",
                    parameters=[
                        {
                            "publish_rate": publish_rate,
                            "step_duration": step_duration,
                            "fixed_grasp_wrist_roll_rad": fixed_grasp_wrist_roll_rad,
                        }
                    ],
                    output="screen",
                )
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config_path)],
            condition=IfCondition(start_rviz),
            output="screen",
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
                "publish_rate",
                default_value="30.0",
                description="Simulated /joint_states publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "step_duration",
                default_value="1.5",
                description="Seconds spent interpolating between grasp waypoints.",
            ),
            DeclareLaunchArgument(
                "sim_start_delay",
                default_value="5.0",
                description=(
                    "Seconds to let the real camera detector update "
                    "block_location.json before starting the simulated grasp."
                ),
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
                "start_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz2 with the SO101 config.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
