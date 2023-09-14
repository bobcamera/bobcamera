#!/bin/bash

# source options are "'rtsp'", "'usb'", "'video'", "'simulate'"
export BOB_SOURCE="'simulate'"
export BOB_RTSP_URL="rtsp://sky360:Sky360Sky!@10.20.30.75:554/cam/realmonitor?channel=1&subtype=0"
export BOB_RTSP_WIDTH="1920"
export BOB_RTSP_HEIGHT="1080"
export BOB_CAMERA_ID="0"
export BOB_ENABLE_VISUALISER="True"
export BOB_OPTIMISED="True"

./launcher.sh application_launch.py
