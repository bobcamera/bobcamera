#!/bin/bash
source /workspaces/bobcamera/src/ros2/install/setup.bash
source /opt/ros2_ws/install/setup.bash

# Fast DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/bobcamera/src/ros2/config/fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
exec "$@"
