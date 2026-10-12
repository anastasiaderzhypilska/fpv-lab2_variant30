#!/bin/bash

source "$HOME/.bashrc"

export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"

export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

echo "Запускаємо Gazebo Server..."
gz sim -s -r -v4 iris_runway.sdf &
SERVER_PID=$!

sleep 5

echo "Запускаємо Gazebo GUI..."
gz sim -g --render-engine-gui ogre &
GUI_PID=$!

wait "$SERVER_PID" "$GUI_PID"
