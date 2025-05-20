#!/bin/bash

if [ -z "$1" ]; then
  echo "Error: No YAML file provided."
  echo "Usage: $0 <YAML FILE>"
  exit 1
fi

# Log level for Bob to help with debugging
# Options: DEBUG, INFO, WARN, ERROR or FATAL
export BOB_LOGLEVEL=${BOB_LOGLEVEL:-"INFO"}
export ROS_LOG_DIR=./app_log

# Creating directories and copying wsdl files
mkdir -p ./assets/config
mkdir -p ./assets/recordings
mkdir -p ./assets/masks
mkdir -p ./assets/wsdl
cp -r ./install/bob_monitor/share/bob_monitor/ver10 ./assets/wsdl
cp -r ./install/bob_monitor/share/bob_monitor/ver20 ./assets/wsdl

# Installing the current config in the right place
cp $1 ./assets/config/app_config.yaml

# Launch the application
./ros_launcher.sh dynamic_launch.py
