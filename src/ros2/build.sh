#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ros2_ws/install/setup.bash
colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release


