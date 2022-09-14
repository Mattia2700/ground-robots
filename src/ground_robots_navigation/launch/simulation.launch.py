# Author: Addison Sears-Collins
# Date: September 2, 2021
# Description: Launch a basic mobile robot using the ROS 2 Navigation Stack
# https://automaticaddison.com
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

##
# This launch file is used to launch the simulation nodes
def generate_launch_description():
 
  # Set the path to different files and folders.
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_navigation = FindPackageShare(package='ground_robots_navigation').find('ground_robots_navigation')
  pkg_models = FindPackageShare(package='ground_robots_models').find('ground_robots_models')
  pkg_nav2 = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
  pkg_bt = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')

  nav2_launch_dolder = os.path.join(pkg_nav2, 'launch') 

  behavior_tree_xml_path = os.path.join(pkg_bt, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

  static_map_path = os.path.join(pkg_navigation, 'maps', 'povo.yaml')
  nav2_params_path = os.path.join(pkg_navigation, 'params', 'nav2_params.yaml')

  model_path = os.path.join(pkg_models, 'models/robot_model.urdf')
  world_path = os.path.join(pkg_models, 'worlds/povo.world')
   
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
   
  # Declare the launch arguments  
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')
 
  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='False',
    description='Whether to apply a namespace to the navigation stack')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='False',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
    
  # Specify the actions
 
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world, 'verbose': 'true'}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
    launch_arguments={'verbose': 'true'}.items())
 
  
 
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
 
  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
 
  return ld