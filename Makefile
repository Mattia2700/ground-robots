SHELL := /bin/bash

home = ${HOME}
current_dir = ${PWD}

all: clean models ros2 ros1

models:	
	@echo "Creating symbolic link for model in $(home)/.gazebo/models"
	@if [ -d $(home)/.gazebo/models ]; then if (file $(home)/.gazebo/models | grep -q symbolic; echo $? ); then rm -rf $(home)/.gazebo/models; fi; fi;
	@if [ ! -L $(home)/.gazebo/models ] || [ ! -d $(home)/.gazebo/models ]; then if [ ! -d $(home)/.gazebo/ ]; then mkdir -p $(home)/.gazebo; fi; ln -s $(current_dir)/models $(home)/.gazebo/; fi;

ros2:
	@echo "Building ROS2 packages"
	@unset ROS_DISTRO
	@source /opt/ros/foxy/setup.bash
	@colcon build --packages-ignore ros1_bridge
ros1:
	@echo "Building ROS1 packages"
	@cp -r install install_rviz 2>/dev/null
	@unset ROS_DISTRO
	@source /opt/ros/noetic/setup.bash
	@source /opt/ros/foxy/setup.bash
	@source $(home)/workspace1/devel/setup.bash
	@source $(home)/workspace2/install/setup.bash
	@colcon build --packages-select ros1_bridge 

clean:
	@echo "Cleaning build directories"
	@rm -rf $(current_dir)/build $(current_dir)/install $(current_dir)/install_rviz $(current_dir)/log

.PHONY: clean models ros2 ros1
