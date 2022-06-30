FROM osrf/ros:noetic-desktop-full

# select bash as default shell
SHELL ["/bin/bash", "-c"]

# install catkin and rosdep
RUN apt update && apt install python3-catkin-tools python3-rosdep git python3-vcstool -y
RUN source /opt/ros/noetic/setup.bash

# Set the working directory
WORKDIR /usr/src

RUN mkdir -p swerve_ws/src/swerve_base
WORKDIR /usr/src/swerve_ws

RUN catkin init && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
WORKDIR /usr/src/swerve_ws/src
COPY . ./swerve_base/
RUN vcs import . < swerve_base/dependencies_exact.rosinstall --recursive --shallow
RUN rosdep update && rosdep install --from-paths . --ignore-src -r -y
RUN catkin build swerve_control

#source workspace
WORKDIR /usr/src/swerve_ws
