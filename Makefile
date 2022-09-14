SHELL := /bin/bash

home = ${HOME}
current_dir = ${PWD}

models:	
	@echo "Creating symbolic link for model in $(home)/.gazebo/models"
	@if [ -d $(home)/.gazebo/models ]; then if (file $(home)/.gazebo/models | grep -q symbolic; echo $? ); then rm -rf $(home)/.gazebo/models; fi; fi;
	@if [ ! -d $(home)/.gazebo/ ]; then mkdir -p $(home)/.gazebo; fi;
	@if [ ! -L $(home)/.gazebo/models ] || [ ! -d $(home)/.gazebo/models ]; then ln -s $(current_dir)/src/ground_robots_models/models $(home)/.gazebo/; fi;

clean:
	@echo "Cleaning build directories"
	@rm -rf $(current_dir)/build $(current_dir)/install $(current_dir)/log

build:
	@echo "Building ROS2 packages"
	@make msgs
	@source $(current_dir)/install/setup.bash && colcon build --packages-ignore planning_bridge_msgs --parallel-workers $(shell nproc)

navigation: 
	@echo "Running navigation"
	@make models
	@source $(current_dir)/install/setup.bash && ros2 launch ground_robots_navigation navigation.launch.py \
		use_lidar:=True

simulation:
	@echo "Running simulation"
	@make models
	@source $(current_dir)/install/setup.bash && ros2 launch ground_robots_navigation simulation.launch.py \
		use_simulator:=True \
		headless:=True

slam:
	@echo "Running SLAM"
	@make models
	@source $(current_dir)/install/setup.bash && ros2 launch ground_robots_navigation navigation.launch.py \
		slam:=True \
		use_sim_time:=True 

lidar:
	@echo "Running LIDAR transform"
	@source $(current_dir)/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0.45 0 0 0 base_link base_laser

footprint:
	@echo "Running FOOTPRINT transform"
	@source $(current_dir)/install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint

msgs:
	@source /opt/ros/foxy/setup.bash && colcon build --packages-select planning_bridge_msgs

planning:
	@echo "Running planning"
	@source $(current_dir)/install/setup.bash && ros2 launch planning_bridge planning.launch.py

joy:
	@echo "Running joystick"
	@source $(current_dir)/install/setup.bash && ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

.PHONY: models clean build msgs navigation sim bridge planning

