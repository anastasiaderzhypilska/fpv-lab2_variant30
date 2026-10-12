#!/bin/bash

source "$HOME/.bashrc"
source "$HOME/ardu_ws/install/setup.bash"

cd "$HOME/ardu_ws/src/ardupilot" || exit 1

sim_vehicle.py -v ArduCopter -f gazebo-iris \
  --model JSON \
  --console \
  --no-rebuild \
  --add-param-file="$HOME/ardu_ws/src/ardupilot/Tools/ros2/ardupilot_sitl/config/default_params/dds_udp.parm"
