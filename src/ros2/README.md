# ROS2 UAP Detection Pipeline

# Overview

## Processing Stage 1 - Kernel:

* Step 1: Video acquisition and feeding into processing pipeline
* Step 2: Background subtraction (default = ViBe) is applied
* Step 3: Blobs are detected and bounding boxes extracted
* Step 4: Tracker (default = SORT) is initialised using bounding boxes which is then tracked across frames.
* Step 5: Recording is done using whatever mechanism is specified
* Step 6: Annotated frame is constructed for display
* Step 7: Frames are resized and transferred externally for Stage 2 processing

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

#### Update RTSP cam details if you are using an RTSP camera

* copy the example_config.yaml to a file of your on: `cp example_config.yaml rtsp_config.yaml`
* update the camera uri in your yaml file in the camera node: `rtsp_uri: 'YOUR_RTSP_URI'`
* set the source variable to be `source_type: 'RTSP_STREAM'`

#### Update USB cam details if you are using a USB camera

* copy the example_config.yaml to a file of your on: `cp example_config.yaml rtsp_config.yaml`
* update the variables for the camera id in your yaml file: `camera_id: 0`
* set the BOB_SOURCE environment variable to be `source_type: 'USB_CAMERA'`

#### Build

Run the build script to build the application: `./build.sh`

#### Launch

Run the application launch script to launch the application: `./launch_dynamic.sh <YAML FILE>`

#### Note: If you run into an error similar to the following:

`at.apa.xcb: could not connect to display 1 at.apa.plugin: Could not load the Ot platform plugin "xcb" in "" even though it was found`

Run the following command in a standard linux terminal: `xhost +`
Run the following commands in the terminal in vscode i.e. inside the container: `export DISPLAY=:0` (or whatever display index your motor is attached to)

### Interacting with the application

#### Checking the application status via command line using the terminal in vscode

* To keep an eye on the state of the tracker, use the following echo command: `ros2 topic echo /bob/tracker/tracking_state`
* To keep an eye on the camera frame excl. array data use the following echo command: `ros2 topic echo  /bob/frames/annotated/resized --no-arr`