from ast import arguments
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_povo_world = get_package_share_directory("povo_world")
    pkg_grobot_description = get_package_share_directory("g_robot_description")

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        )
    )

    # Robot spawn
    robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-file', os.path.join(pkg_grobot_description, 'config', 'g_robot.urdf'),
            '-entity', 'GRobot'
        ]
    )

    # Controller
    controller = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        respawn=False,
        arguments=[
            '-p', os.path.join(pkg_grobot_description, 'config', 'diff_drive.yaml'), 'g_robot_controller',
            '--controller-manager-timeout', '20' 
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=[
                    os.path.join(pkg_povo_world, "worlds", "empty_world.world")
                ],
                description="SDF world file"
            ),
            gazebo,
            robot,
            controller
        ]
    )
