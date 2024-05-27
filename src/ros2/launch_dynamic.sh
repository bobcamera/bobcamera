#!/bin/bash

# Log level for Bob to help with debugging
# Options: DEBUG, INFO, WARN, ERROR or FATAL
export BOB_LOGLEVEL=${BOB_LOGLEVEL:-"INFO"}

# Launch the application
./launcher.sh dynamic_launch.py
