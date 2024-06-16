#!/bin/bash

# Log level for Bob to help with debugging
# Options: DEBUG, INFO, WARN, ERROR or FATAL
export BOB_LOGLEVEL=${BOB_LOGLEVEL:-"INFO"}
export RCUTILS_LOGGING_FILE_NAME=./bob.log

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
./launcher.sh dynamic_launch.py
