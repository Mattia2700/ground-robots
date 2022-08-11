SHELL := /bin/bash

home = ${HOME}
current_dir = ${PWD}

ifeq ($(SIM),)
all: clean build run
else
all: clean build sim
endif

clean:
	@echo "Cleaning build directories"
	@rm -rf $(current_dir)/build $(current_dir)/install $(current_dir)/log

build:
	@echo "Building ROS2 packages"
	@clear
	@source /opt/ros/foxy/setup.bash
	@make msgs
	@colcon build --packages-ignore waypoint_msgs

run: 
	@echo "Running navigation"
	@source $(current_dir)/install/setup.bash && ros2 launch g_robot navigation.launch.py

sim:
	@echo "Running simulation"
	@source $(current_dir)/install/setup.bash && ros2 launch g_robot navigation.launch.py \
		use_simulator:=True \
		headless:=True \
		rviz_config_file:='$(current_dir)/install/g_robot/share/g_robot/rviz/nav2_config_sim.rviz'

msgs:
	@if [ ! -d $(current_dir)/install ]; then echo "Building messages"; colcon build --packages-select waypoint_msgs; fi

bridge:
	@echo "Running bridge"
	@source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics

.PHONY: clean build models run sim
