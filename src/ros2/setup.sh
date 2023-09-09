#!/bin/bash
set -e

vcs import < src/ros2.repos src

rosdep update
rosdep install --from-paths src --ignore-src -y