# BobCamera Web GUI

## Overview

This repository contains the code for the BobCamera Web GUI. It is designed to work alongside a ROS2 environment.

## Requirements

- Docker
- VS Code
- VS Code Docker Extension

## Getting Started

### Initial Setup

1. **Clone the repository:**
    ```bash
    git clone https://github.com/bobcamera/bobcamera.git
    ```

2. **Navigate to the web application directory:**
    ```bash
    cd bobcamera/src/web2
    ```

3. **Ensure Docker is running:**
    Make sure the Docker application is running on your machine (it can be in the background).

4. **Open the directory in VS Code:**
    ```bash
    code .
    ```

5. **Reopen in Docker Container:**
    Using the VS Code Docker Extension, reopen the directory in a Docker Container.

### Launching the Web GUI

1. **Click Run --> Start Debugging || Run Without Debugging from the VS Code menu**

2. **Access the Web GUI:**
    Step number 1 should launch a browser session pointing you to the Web GUI.    

**Note**: If you have a port conflict.

  1. Browse to the "PORTS" tab, part of the terminal window set
  2. Select the port entry causing the conflict and press the "delete" key

## Connecting to ROS2 Environment

**Note**: Follow these steps only if you haven't already launched the ROS2 container.

1. **Navigate to the ROS2 directory:**
    ```bash
    cd bobcamera/src/ros
    ```

2. **Launch the ROS2 environment:**
    Use the `.devcontainer` configuration in this folder to launch the ROS2 environment. For detailed instructions, consult the ROS2 directory README.

**Note**:
If you find there is no live stream or broken functionality on pages in the web GUI, then it could be that port 9090 used by ROSBridge web server is not forwarded correctly within VS Code

### Additional Resources

- [BobCamera ROS2 Source Code](https://github.com/bobcamera/bobcamera/tree/main/src/ros2)
