FROM sky360/ros2-dev:latest
ARG USERNAME=ros
RUN apt-get update && apt-get -y install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && echo "export CYCLONEDDS_URI=file:///workspaces/sky360/src/ros2/cyclonedds.xml" >> /home/$USERNAME/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc 
