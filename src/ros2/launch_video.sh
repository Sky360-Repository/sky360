#!/bin/bash
source install/setup.bash
pkill -9 iox-roudi
iox-roudi -c config/roudi.toml &
ros2 launch sky360_launch video_launch.py
pkill -9 iox-roudi