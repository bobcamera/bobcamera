#!/bin/bash

# source options are "'rtsp'", "'usb'", "'video'", "'simulate'", "'rtsp_overlay'"
export BOB_SOURCE=${BOB_SOURCE:-"'video'"}
export BOB_RTSP_URL=${BOB_RTSP_URL:-"rtsp://sky360:Sky360Sky!@10.20.30.75:554/cam/realmonitor?channel=1&subtype=0"}
export BOB_RTSP_WIDTH=${BOB_RTSP_WIDTH:-"1920"}
export BOB_RTSP_HEIGHT=${BOB_RTSP_HEIGHT:-"1080"}
export BOB_CAMERA_ID=${BOB_CAMERA_ID:-"0"}
export BOB_ENABLE_VISUALISER=${BOB_ENABLE_VISUALISER:-"True"}
export BOB_OPTIMISED=${BOB_OPTIMISED:-"True"}
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/bobcamera/src/ros2/config/fastdds.xml
# export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export RMW_IMPLEMENTATION={RMW_IMPLEMENTATION:-"rmw_fastrtps_cpp"}
export FASTRTPS_DEFAULT_PROFILES_FILE={FASTRTPS_DEFAULT_PROFILES_FILE:-"/workspaces/bobcamera/src/ros2/config/fastdds.xml"}
export BOB_BGS_ALGORITHM=${BOB_BGS_ALGORITHM:-"vibe"} # or wmv
export BOB_VIBE_PARAMS=${BOB_VIBE_PARAMS:-"{\"threshold\": 50, \"bgSamples\": 16, \"requiredBGSamples\": 2, \"learningRate\": 4}"} 
export BOB_WMV_PARAMS=${BOB_WMV_PARAMS:-"{\"enableWeight\": true, \"enableThreshold\": true, \"threshold\": 25.0, \"weight1\": 0.5, \"weight2\": 0.3, \"weight3\": 0.2}"} 
export BOB_BLOB_PARAMS=${BOB_BLOB_PARAMS:-"{\"sizeThreshold\": 2, \"areaThreshold\": 2, \"minDistance\": 4, \"maxBlobs\": 100}"} 

./launcher.sh application_launch.py
