#!/bin/bash
source install/setup.bash

rnw="${2:-fastdds}"

if [ "$rnw" == "fastdds" ]; then
    # Fast DDS
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/sky360/src/ros2/config/fastdds.xml
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
    ros2 launch sky360_launch $1
else
    # Cyclone DDS
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///workspaces/sky360/src/ros2/config/cyclonedds.xml
    pkill -9 iox-roudi
    iox-roudi -c config/roudi.toml &
    ros2 launch sky360_launch $1
    pkill -9 iox-roudi
fi