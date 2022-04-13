FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y python3-colcon-common-extensions
RUN apt-get install -y ros-foxy-test-msgs 
RUN apt-get install -y ros-foxy-gazebo-ros-pkgs 
RUN apt-get install -y ros-foxy-xacro
RUN apt-get install -y ros-foxy-joint-state-publisher-gui
RUN apt-get update && apt-get install -y ros-foxy-rqt-robot-steering
RUN apt-get install -y ros-foxy-nav2-bringup
RUN apt-get install -y ros-foxy-robot-localization

RUN sed -i 's/(ALL:ALL) ALL/(ALL) NOPASSWD: ALL/' /etc/sudoers # Enable sudo without password

# Add user with home folder
RUN useradd -ms /bin/bash -G sudo ros2
USER ros2
WORKDIR /home/ros2

RUN echo 'alias rosplz="source ~/workspace/install/setup.bash"' >> ~/.bashrc
RUN echo 'alias rosinstall="rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy"' >> ~/.bashrc
RUN echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=~/workspace" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=~/workspace/models/gazebo:~/workspace/src/g_robot/models" >> ~/.bashrc
#export GAZEBO_MODEL_DATABASE_URI=""

ENTRYPOINT ["bash"]
