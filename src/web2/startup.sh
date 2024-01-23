#!/bin/bash

# Start Apache in the background
service apache2 start

# If you have other commands to run, add them here

# Keep the container running
tail -f /dev/null
