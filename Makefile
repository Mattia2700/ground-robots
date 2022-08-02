SHELL := /bin/bash

home = ${HOME}
current_dir = ${PWD}

ifeq ($(SIM),)
all: clean build run
else
all: clean build models sim
endif

clean:
	@echo "Cleaning build directories"
	@rm -rf $(current_dir)/build $(current_dir)/install $(current_dir)/log

build:
	@echo "Building ROS2 packages"
	@clear
	@source /opt/ros/foxy/setup.bash
	@colcon build

models:	
	@echo "Creating symbolic link for model in $(home)/.gazebo/models"
	@if [ -d $(home)/.gazebo/models ]; then if (file $(home)/.gazebo/models | grep -q symbolic; echo $? ); then rm -rf $(home)/.gazebo/models; fi; fi;
	@if [ ! -d $(home)/.gazebo/ ]; then mkdir -p $(home)/.gazebo; fi;
	@if [ ! -L $(home)/.gazebo/models ] || [ ! -d $(home)/.gazebo/models ]; then ln -s $(current_dir)/models $(home)/.gazebo/; fi;	

run: 
	@echo "Running navigation"
	@source $(current_dir)/install/setup.bash && ros2 launch g_robot navigation.launch.py

sim:
	@echo "Running simulation"
	@source $(current_dir)/install/setup.bash && ros2 launch g_robot navigation.launch.py use_simulator:=True use_robot_state_pub:=True rviz_config_file:='$(current_dir)/install/g_robot/share/g_robot/rviz/nav2_config_sim.rviz'

bridge:
	@echo "Running bridge"
	@source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics

.PHONY: clean build models run sim
