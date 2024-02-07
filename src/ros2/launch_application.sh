#!/bin/bash

# NOTE: Please ensure that you maintain the format and syntax of this launch script when making modifications.
# Any changes should follow the existing structure to avoid errors or unexpected behavior.

# Source options: rtsp, usb, video, simulate, rtsp_overlay, video_overlay
export BOB_SOURCE=${BOB_SOURCE:-"'video_overlay'"}

# RTSP URL examples:
# Hikvision: export BOB_RTSP_URL=${BOB_RTSP_URL:-"rtsp://user:password@0.0.0.0:554/Streaming/Channels/101"}
# Amcrest and Dahua: export BOB_RTSP_URL=${BOB_RTSP_URL:-"rtsp://user:password@0.0.0.0:554/cam/realmonitor?channel=1&subtype=0"}
export BOB_RTSP_URL=${BOB_RTSP_URL:-""}

export BOB_CAMERA_ID=${BOB_CAMERA_ID:-"0"}
export BOB_ENABLE_VISUALISER=${BOB_ENABLE_VISUALISER:-"True"}
export BOB_ENABLE_RECORDING=${BOB_ENABLE_RECORDING:-"False"}

# RMW settings
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-"rmw_fastrtps_cpp"}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-"/workspaces/bobcamera/src/ros2/config/fastdds.xml"}

# Background subtraction algorithm: vibe or wmv
export BOB_BGS_ALGORITHM=${BOB_BGS_ALGORITHM:-"vibe"}

# Tracking sensitivity options: minimal, low, medium, high
export BOB_TRACKING_SENSITIVITY=${BOB_TRACKING_SENSITIVITY:-"'medium'"}

# Masking
export BOB_TRACKING_USEMASK=${BOB_TRACKING_USEMASK:-"False"}
export BOB_TRACKING_MASK_FILE=${BOB_TRACKING_MASK_FILE:-"assets/masks/mask.jpg"}

# Video files list separated by ';'
export BOB_VIDEOS=${BOB_VIDEOS:-"/workspaces/bobcamera/test/fisheye_videos/mike_drone.mp4;/workspaces/bobcamera/test/fisheye_videos/Dahua-20220901-184734.mp4;/workspaces/bobcamera/test/fisheye_videos/brad_drone_1.mp4"}

# Simulation settings
export BOB_SIMULATION_NUM_OBJECTS=${BOB_SIMULATION_NUM_OBJECTS:-"5"}

# Launch the application
./launcher.sh application_launch.py
