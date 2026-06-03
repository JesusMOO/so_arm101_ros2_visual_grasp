from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    calibration = LaunchConfiguration('calibration').perform(context)
    serial_port = LaunchConfiguration('serial_port')
    publish_rate = LaunchConfiguration('publish_rate')
    use_hardware_joint_state_publisher = LaunchConfiguration(
        'use_hardware_joint_state_publisher')
    use_generic_joint_state_publisher = LaunchConfiguration(
        'use_generic_joint_state_publisher')
    use_gui_joint_state_publisher = LaunchConfiguration(
        'use_gui_joint_state_publisher')
    package_share = Path(get_package_share_directory('lerobot'))
    urdf_path = package_share / 'urdf' / 'so101' / f'so101_{calibration}_calib.urdf'
    rviz_config_path = package_share / 'rviz' / 'so101.rviz'

    robot_description = urdf_path.read_text(encoding='utf-8')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[str(urdf_path)],
            condition=IfCondition(use_generic_joint_state_publisher),
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[str(urdf_path)],
            condition=IfCondition(use_gui_joint_state_publisher),
            output='screen',
        ),
        Node(
            package='lerobot',
            executable='so101_joint_state_publisher_node',
            name='so101_joint_state_publisher',
            parameters=[{
                'serial_port': serial_port,
                'publish_rate': publish_rate,
            }],
            condition=IfCondition(use_hardware_joint_state_publisher),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_config_path)],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration',
            default_value='new',
            choices=['new', 'old'],
            description='SO101 calibration model to display.',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for the SO101 ST3215 servo bus.',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='20.0',
            description='Joint state publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'use_hardware_joint_state_publisher',
            default_value='true',
            choices=['true', 'false'],
            description='Publish /joint_states from the real SO101 ST3215 servo readings.',
        ),
        DeclareLaunchArgument(
            'use_generic_joint_state_publisher',
            default_value='false',
            choices=['true', 'false'],
            description='Publish /joint_states from the generic joint_state_publisher.',
        ),
        DeclareLaunchArgument(
            'use_gui_joint_state_publisher',
            default_value='false',
            choices=['true', 'false'],
            description='Publish /joint_states from joint_state_publisher_gui sliders.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
