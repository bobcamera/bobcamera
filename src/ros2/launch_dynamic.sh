#!/bin/bash

# Log level for Bob to help with debugging
# Options: DEBUG, INFO, WARN, ERROR or FATAL
export BOB_LOGLEVEL=${BOB_LOGLEVEL:-"INFO"}
export RCUTILS_LOGGING_FILE_NAME=./bob.log

cp ./src/bob_launch/config/$1 ./assets/config/app_config.yaml

# Launch the application
./launcher.sh dynamic_launch.py
