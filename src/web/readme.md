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
    cd bobcamera/src/web
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

1. **Open a new terminal window in VS Code:**
    Once your workspace has reopened in the Docker container, open a new terminal window inside VS Code.

2. **Run the `run_application.sh` script:**
    ```bash
    ./run_application.sh
    ```

3. **Access the Web GUI:**
    After running the script, a prompt will appear in VS Code asking if you'd like to "Open in Browser". Click it to access the web GUI. If you miss the prompt, manually open your web browser and navigate to `http://localhost:8080`.

## Connecting to ROS2 Environment

> **Note**: Follow these steps only if you haven't already launched the ROS2 container.

1. **Navigate to the ROS2 directory:**
    ```bash
    cd bobcamera/src/ros
    ```

2. **Launch the ROS2 environment:**
    Use the `.devcontainer` configuration in this folder to launch the ROS2 environment. For detailed instructions, consult the ROS2 directory README.

### Additional Resources

- [BobCamera ROS2 Source Code](https://github.com/bobcamera/bobcamera/tree/main/src/ros2)
