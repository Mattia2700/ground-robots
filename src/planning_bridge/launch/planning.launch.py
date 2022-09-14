import os
from launch import LaunchDescription
from launch_ros.actions import Node

##
# This launch file is used to launch the planning bridge nodes 
def generate_launch_description():

    start_navigation_client = Node(
        package='planning_bridge',
        executable='navigation_client',
        output='screen')

    start_pose_server = Node(
        package='planning_bridge',
        executable='pose_server',
        output='screen')

    start_ugv_move = Node(
        package='planning_bridge',
        executable='ugv_move',
        output='screen')

    start_ugv_transporting_uav_move = Node(
        package='planning_bridge',
        executable='ugv_transporting_uav_move',
        output='screen')


    ld = LaunchDescription()
    ld.add_action(start_navigation_client)
    ld.add_action(start_pose_server)
    ld.add_action(start_ugv_move)
    ld.add_action(start_ugv_transporting_uav_move)
 
    return ld