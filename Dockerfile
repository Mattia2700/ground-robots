FROM osrf/ros:foxy-ros1-bridge

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND noninteractive
ENV QT_X11_NO_MITSHM 1

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654


RUN \
  apt-get update && \
  apt-get -y install apt-utils

RUN \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri mesa-utils dbus 

RUN \
  apt-get -y install nano python3-pip libzmq3-dev
  
RUN \
  apt-get -y install ros-noetic-tf2-bullet \
                     ros-noetic-grid-map-ros \
                     ros-noetic-voxel-grid \
                     ros-noetic-amcl \
                     ros-noetic-costmap-2d \
                     ros-noetic-map-server \
                     ros-noetic-depthimage-to-laserscan \
                     ros-noetic-slam-toolbox --fix-missing
                     
RUN \
  apt-get -y upgrade


RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y python3-colcon-common-extensions
RUN apt-get install -y ros-foxy-test-msgs 
RUN apt-get install -y ros-foxy-gazebo-ros-pkgs 
RUN apt-get install -y ros-foxy-xacro
RUN apt-get install -y ros-foxy-joint-state-publisher-gui
RUN apt-get update && apt-get install -y ros-foxy-rqt-robot-steering
RUN apt-get install -y ros-foxy-nav2-bringup
RUN apt-get install -y ros-foxy-robot-localization
RUN apt-get install -y ros-foxy-lifecycle-msgs

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
RUN echo "export GAZEBO_MODEL_PATH=~/workspace2/src/g_robot/models" >> ~/.bashrc
#export GAZEBO_MODEL_DATABASE_URI=""

RUN sudo apt-get install -y file ros-foxy-rviz2

RUN sudo apt-get update && sudo apt-get upgrade -y

ENTRYPOINT ["bash"]
