#!/bin/bash
source /workspaces/bobcamera/src/ros2/install/setup.bash
source /opt/ros2_ws/install/setup.bash

# Fast DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/bobcamera/src/ros2/config/fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_LOG_DIR=/workspaces/bobcamera/app_log

# Creating directories and copying wsdl files
mkdir -p /workspaces/bobcamera/src/ros2/assets/config
mkdir -p /workspaces/bobcamera/src/ros2/assets/recordings
mkdir -p /workspaces/bobcamera/src/ros2/assets/masks
mkdir -p /workspaces/bobcamera/src/ros2/assets/wsdl
cp -r /workspaces/bobcamera/src/ros2/install/bob_monitor/share/bob_monitor/ver10 /workspaces/bobcamera/src/ros2/assets/wsdl
cp -r /workspaces/bobcamera/src/ros2/install/bob_monitor/share/bob_monitor/ver20 /workspaces/bobcamera/src/ros2/assets/wsdl

# Fix rosbridge line endings and symlink
cd /workspaces/bobcamera/src/ros2/install/rosbridge_server/lib/rosbridge_server
rm -f rosbridge_websocket
ln -s rosbridge_websocket.py rosbridge_websocket
sed -i 's/\r$//' rosbridge_websocket.py
find /workspaces/bobcamera/src/ros2/install/rosapi/lib/rosapi/ -type f -name 'rosapi_node*' -exec sed -i 's/\r$//' {} \;

exec "$@"
