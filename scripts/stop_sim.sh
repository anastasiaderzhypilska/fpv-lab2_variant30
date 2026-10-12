#!/bin/bash

echo "Зупиняємо ArduPilot SITL..."
pkill -f '[a]rducopter' || true
pkill -f '[s]im_vehicle.py' || true
pkill -f '[m]avproxy.py' || true

echo "Зупиняємо Gazebo..."
pkill -f '[g]z sim' || true
pkill -f '[g]z-sim-server' || true
pkill -f '[g]z-sim-gui' || true

echo "Зупиняємо DDS Agent..."
pkill -f '[m]icro_ros_agent' || true

echo "Симуляцію зупинено."
