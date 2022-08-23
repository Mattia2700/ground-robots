SHELL := /bin/bash

home = ${HOME}
current_dir = ${PWD}

ifeq ($(SIM),)
all: clean build run
else
all: clean build sim
endif

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
	@clear
	@source /opt/ros/foxy/setup.bash
	@make msgs
	@source $(current_dir)/install/setup.bash && colcon build --packages-ignore planning_bridge_msgs

run: 
	@echo "Running navigation"
	@source $(current_dir)/install/setup.bash && ros2 launch ground_robots_navigation navigation.launch.py

sim:
	@echo "Running simulation"
	@make models
	@source $(current_dir)/install/setup.bash && ros2 launch ground_robots_navigation navigation.launch.py \
		use_simulator:=True \
		headless:=True \
		rviz_config_file:='$(current_dir)/install/ground_robots_navigation/share/ground_robots_navigation/rviz/nav2_config_sim.rviz'

msgs:
	@colcon build --packages-select planning_bridge_msgs

bridge:
	@echo "Running bridge"
	@source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics

planning:
	@echo "Running planning"
	@source $(current_dir)/install/setup.bash && ros2 launch planning_bridge planning.launch.py

.PHONY: clean build models run sim

