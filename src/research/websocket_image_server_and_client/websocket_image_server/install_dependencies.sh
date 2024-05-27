#!/bin/bash

# Update the package list
sudo apt-get update

# Install Python 3 and pip3 if not already installed
sudo apt-get install -y python3 python3-pip

# Install the picamera library
sudo apt-get install -y python3-picamera

# Install the websockets library
pip3 install websockets

# Enable the camera interface
sudo raspi-config nonint do_camera 0

# Prompt to reboot the Raspberry Pi
echo "The camera interface has been enabled. A reboot is required for changes to take effect."
read -p "Would you like to reboot now? (y/n): " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo reboot
else
    echo "Please remember to reboot your Raspberry Pi later to apply the changes."
fi

# Instructions to run the Python script
echo "Dependencies installed successfully."
echo "Save your Python script (camera_stream.py) to a desired location."
echo "Navigate to the directory where the script is saved and run the following command to start the server:"
echo "python3 camera_stream.py"
