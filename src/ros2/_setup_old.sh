#!/bin/bash
set -e

# TODO: Figure why we need to be running this here
sudo apt-get update
sudo apt-get install --fix-missing
# install gstreamer - move this into the container
#sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
# End TODO:

#vcs import < src/ros2.repos src

#rosdep update
#rosdep install --from-paths src --ignore-src -y

pip install onvif2_zeep