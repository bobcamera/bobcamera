#!/bin/bash
source /workspaces/bobcamera/src/ros2/install/setup.bash
source /opt/ros2_ws/install/setup.bash

# Fast DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/bobcamera/src/ros2/config/fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_LOG_DIR=/workspaces/bobcamera/src/ros2/app_log

# Creating directories and copying wsdl files
mkdir -p /workspaces/bobcamera/src/ros2/assets/config
mkdir -p /workspaces/bobcamera/src/ros2/assets/recordings
mkdir -p /workspaces/bobcamera/src/ros2/assets/masks
mkdir -p /workspaces/bobcamera/src/ros2/assets/wsdl
cp -r /workspaces/bobcamera/src/ros2/install/bob_monitor/share/bob_monitor/ver10 /workspaces/bobcamera/src/ros2/assets/wsdl
cp -r /workspaces/bobcamera/src/ros2/install/bob_monitor/share/bob_monitor/ver20 /workspaces/bobcamera/src/ros2/assets/wsdl

exec "$@"
