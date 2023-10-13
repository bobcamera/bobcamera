#!/bin/bash

# source options are "'rtsp'", "'usb'", "'video'", "'simulate'", "'rtsp_overlay'", "'video_overlay'"
export BOB_SOURCE=${BOB_SOURCE:-"'video_overlay'"}
#export BOB_RTSP_URL=${BOB_RTSP_URL:-"rtsp://sky360:Sky360Sky!@10.20.30.75:554/cam/realmonitor?channel=1&subtype=0"}
export BOB_RTSP_WIDTH=${BOB_RTSP_WIDTH:-"1920"}
export BOB_RTSP_HEIGHT=${BOB_RTSP_HEIGHT:-"1080"}
export BOB_RTSP_URL=${BOB_RTSP_URL:-"rtsp://bob:Sky360Sky!@10.20.30.58:554/Streaming/Channels/101"}
#export BOB_RTSP_WIDTH=${BOB_RTSP_WIDTH:-"2560"}
#export BOB_RTSP_HEIGHT=${BOB_RTSP_HEIGHT:-"2560"}
export BOB_CAMERA_ID=${BOB_CAMERA_ID:-"0"}
export BOB_ENABLE_VISUALISER=${BOB_ENABLE_VISUALISER:-"True"}
export BOB_OPTIMISED=${BOB_OPTIMISED:-"True"}
export BOB_ENABLE_RECORDING=${BOB_ENABLE_RECORDING:-"False"}
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/bobcamera/src/ros2/config/fastdds.xml
# export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-"rmw_fastrtps_cpp"}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-"/workspaces/bobcamera/src/ros2/config/fastdds.xml"}
export BOB_BGS_ALGORITHM=${BOB_BGS_ALGORITHM:-"vibe"} # or wmv

# sensitivity options are "'minimal'", "'low'", "'medium'", "'high'"
export BOB_TRACKING_SENSITIVITY=${BOB_TRACKING_SENSITIVITY:-"'medium'"}

# masking
export BOB_TRACKING_USEMASK=${BOB_TRACKING_USEMASK:-"False"}
export BOB_TRACKING_MASK_FILE=${BOB_TRACKING_MASK_FILE:-"mask.pgm"}

# simulation
export BOB_SIMULATION_WIDTH=${BOB_SIMULATION_WIDTH:- "1920"}
export BOB_SIMULATION_HEIGHT=${BOB_SIMULATION_HEIGHT:- "1080"}
export BOB_SIMULATION_NUM_OBJECTS=${BOB_SIMULATION_NUM_OBJECTS:- "5"}

./launcher.sh application_launch.py
