FROM ubuntu:20.04

RUN apt-get update && apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8


RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y ros-foxy-desktop

RUN apt-get install -y python3-colcon-common-extensions
RUN apt-get install -y ros-foxy-test-msgs 
RUN apt-get install -y ros-foxy-gazebo-ros-pkgs 
RUN apt-get install -y ros-foxy-xacro
RUN apt-get install -y ros-foxy-joint-state-publisher-gui
RUN apt-get install -y ros-foxy-rqt-robot-steering
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
RUN echo "export GAZEBO_MODEL_PATH=~/workspace/models/gazebo:~/workspace/models/povo:~/workspace/src/g_robot/models" >> ~/.bashrc
#export GAZEBO_MODEL_DATABASE_URI=""

ENTRYPOINT ["bash"]
