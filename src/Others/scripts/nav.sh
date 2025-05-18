#!/bin/bash
source ../install/setup.bash
sleep 0.2
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=test4 \
slam:=False \
use_robot_state_pub:=False
