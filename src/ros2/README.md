# ROS2 UAP Detection Pipeline

# Overview

## Processing Stage 1 - Kernel:

* Step 1: Video acquisition and feeding into processing pipeline
* Step 2: Background subtraction is applied
* Step 3: Blobs are detected and bounding boxes extracted
* Step 4: Trackers (CSRT) are initialised using bounding boxes which is then tracked across frames.
* Step 5: We use a Kalman Filter to try and predict the trajectory of the target being tracked
* Step 6: Recording using RosBag
* Step 7: Annotated frame is constructed for display
* Step 8: Frames are resized and transferred externally for Stage 2 processing

## Processing Stage 2 - Application:

### Observer
* Frames are sampled using a day/night classifier
* Frames are sampled using a cloud/sun percentage classifier

### Monitor
* Tracking state is exposed via a Prometheus end point

### Display
* The annotated frame is displayed

## Prerequisites
Before you proceed, ensure that you have the following:

* ** Windows Users**: Windows Subsystem for Linux 2 (WSL 2) installed on your Windows machine. You can follow the official Microsoft documentation to install WSL 2. Install Ubuntu from the Microsoft Store.
* You will need to have both Git and Docker installed on your system.
* Install Visual Studio Code (VSCode) with the 'Dev Containers' extension.

## Running

### NOTE: The commands below should be used to get this ROS package up and running in VSCode and using the ROS dev container

Open a terminal session:

1. git clone the repository: `git clone https://github.com/bobcamera/bobcamera.git`
2. navigate to the ros2 source location: `cd ./bobcamera/src/ros2`
3. Open vscode by typing: `code .`

#### Setup 

Run the setup script to setup dev container. This is required for rosbridge_suite and any other third party packages like the ros_ipcamera package: `./setup.sh`

#### Update RTSP cam details if you are using an RTSP camera

* update the environment variables for both camera rtsp credentials and frame size in `./launch_application.sh`
* set the BOB_SOURCE environment variable to be `"'rtsp'"`

#### Update USB cam details if you are using a USB camera

* update the environment variables for the camera id in `./launch_application.sh`, default id is 0
* set the BOB_SOURCE environment variable to be `"'usb'"`

#### Build

Run the build script to build the application: `./build.sh`

#### Launch

Run the application launch script to launch the application: `./launch_application.sh`

#### Note: If you run into an error similar to the following:

`at.apa.xcb: could not connect to display 1 at.apa.plugin: Could not load the Ot platform plugin "xcb" in "" even though it was found`

Run the following command in a standard linux terminal: `xhost +`
Run the following commands in the terminal in vscode i.e. inside the container: `export DISPLAY=:0` (or whatever display index your motor is attached to)

### Interacting with the application

#### Checking the application status via command line using the terminal in vscode

* To keep an eye on the state of the tracker, use the following echo command: `ros2 topic echo /bob/tracker/tracking_state`
* To keep an eye on the camera frame excl. array data use the following echo command: `ros2 topic echo  /bob/frames/annotated/resized --no-arr`

## TODO:

#### Replay a rosbag recording

* TODO: The annotated frame is published as an rtsp stream for display via other applications e.g. VLC

* `ros2 bag play 2023_09_06-16_48_14/` where 2023_09_06-16_48_14/ is a folder that contains a rosbag recording
* `./launch_rosbag_replay.sh`
