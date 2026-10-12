#!/bin/bash

source /opt/ros/humble/setup.bash
source "$HOME/ardu_ws/install/setup.bash"

ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
