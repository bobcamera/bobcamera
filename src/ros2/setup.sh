#!/bin/bash
set -e

# TODO: Figure why we need to be running this here
sudo apt-get update
sudo apt-get install --fix-missing
# End TODO:

vcs import < src/ros2.repos src

rosdep update
rosdep install --from-paths src --ignore-src -y