#!/bin/bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#pkill -9 iox-roudi
#iox-roudi -c config/roudi.toml &
ros2 launch sky360_launch $1
#pkill -9 iox-roudi