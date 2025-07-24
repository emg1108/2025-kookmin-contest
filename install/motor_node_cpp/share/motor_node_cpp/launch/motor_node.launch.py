import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    motor_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_motor'),
                'launch/xycar_motor.launch.py'))
    )

    motor_publisher_node = Node(
        package='motor_node_cpp',
        executable='motor_node',
        name='motor_node',
        output='screen'
    )

    auto_drive_path = "/home/xytron/osy_250720/2025-SEAME/auto_drive"

    auto_drive_process = ExecuteProcess(
        cmd=[auto_drive_path, "d"],
        output='screen'
    )

    return LaunchDescription([
        motor_include,
        motor_publisher_node,
        auto_drive_process
    ])
