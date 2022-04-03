FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y python3-colcon-common-extensions
RUN apt-get install -y ros-foxy-test-msgs 
RUN apt-get install -y ros-foxy-gazebo-ros-pkgs 
RUN apt-get install -y ros-foxy-xacro

RUN sed -i 's/(ALL:ALL) ALL/(ALL) NOPASSWD: ALL/' /etc/sudoers # Enable sudo without password

# Add user with home folder
RUN useradd -ms /bin/bash -G sudo ros2
USER ros2
WORKDIR /home/ros2

RUN echo 'alias rosplz="source ~/workspace/install/setup.bash"' >> ~/.bashrc
RUN echo 'alias rosinstall="rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy"' >> ~/.bashrc
RUN echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc

ENTRYPOINT ["bash"]
