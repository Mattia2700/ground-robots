SHELL := /bin/bash

home = ${HOME}
current_dir = ${PWD}

all: clean build

models:	
	@echo "Creating symbolic link for model in $(home)/.gazebo/models"
	@if [ -d $(home)/.gazebo/models ]; then if (file $(home)/.gazebo/models | grep -q symbolic; echo $? ); then rm -rf $(home)/.gazebo/models; fi; fi;
	@if [ ! -L $(home)/.gazebo/models ] || [ ! -d $(home)/.gazebo/models ]; then if [ ! -d $(home)/.gazebo/ ]; then mkdir -p $(home)/.gazebo; fi; ln -s $(current_dir)/models $(home)/.gazebo/; fi;

build:
	@echo "Building ROS2 packages"
	@unset ROS_DISTRO
	@source /opt/ros/foxy/setup.bash
	@colcon build

run: 
	ros2 launch g_robot g_robot_v5.launch.py

clean:
	@echo "Cleaning build directories"
	@rm -rf $(current_dir)/build $(current_dir)/install $(current_dir)/install_rviz $(current_dir)/log

.PHONY: clean ros
