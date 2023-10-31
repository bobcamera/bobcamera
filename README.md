# BOB the UAP Detector, Tracker and Recorder

Below are the installation instructions for installing BOB, your friendly neighbourhood UAP detector, tracker and recorder
The application uses Docker and Docker Compose to run, at present we are still in a very early release phase so this the install process is not polished at all. We will endeavour to improve this over time to please bear with us.

## The following steps will need to be performed in a linux terminal

- Ubuntu 23.10 has been tested

### 1. Install dependencies
- Git
- Curl
```
sudo apt-get install git curl
```
### 2. Clone the github repo
```
git clone --recursive https://github.com/bobcamera/bobcamera.git
```
### 3. Change directory to bobcamera
```
cd bobcamera
```
### 4. Set the setup script as executable
```
chmod +x setup.sh
```
### 5. Run the setup script, this will install docker and docker compose
```
./setup.sh
```
### 6. Reboot the machine
```
sudo shutdown -r now
```
### 7. Navigate back to the bobcamera directory once rebooted
```
cd ~/bobcamera
```
### 8. Create the .env file
This is a file that simulates having a bunch of environment variables which are used to configure the BOB tracking system
```
touch .env
```
### 9. Edit the .env file, feel free to use your preferred editor
```
nano .env
```
### 10. Paste this into the .env file (it's a start, you will need to configure it correctly)
```
# Source options are "'rtsp'", "'usb'", "'video'", "'simulate'", "'rtsp_overlay'", "'video_overlay'"
BOB_SOURCE="'rtsp'"
# URL to the rtsp main video stream 
# If you are having trouble, maybe use something like VLC media player to ensure the rtsp url is correct
BOB_RTSP_URL="rtsp://username:password@10.20.30.58:554/Streaming/Channels/101"
# What the rtsp image width is set to 
BOB_RTSP_WIDTH="2560"
# What the rtsp image height is set to 
BOB_RTSP_HEIGHT="2560"
# If you want to use a USB camera, then this is the camera index associated with the camera
BOB_CAMERA_ID="0"
# Do you want to capture recordings i.e. video files of the objects that are being tracked?
BOB_ENABLE_RECORDING="False"
# Back Ground Subtraction algorithm options are "vibe", "wmv"
BOB_BGS_ALGORITHM="vibe"
# We have configured a pre set of tracking sensitivity options, as with everything, these are still work in progress
# Sensitivity options are "'minimal'", "'low'", "'medium'", "'high'"
BOB_TRACKING_SENSITIVITY="'medium'"
# Mask configuration
# This is used for both privacy and motion detection purposes. We encourage you to define a mask as it will significantly reduce noise 
BOB_TRACKING_USEMASK="False"
BOB_TRACKING_MASK_FILE="assets/masks/mask.jpg"
# If you would like to overlay simulation objects over the rtsp stream then you will need to set these
# The source also needs to be set to rtsp_overlay fro this to work
# Simulation section, the width and height need to match that of the frame source
BOB_SIMULATION_WIDTH="2560"
BOB_SIMULATION_HEIGHT="2560"
BOB_SIMULATION_NUM_OBJECTS="5"
# This is mainly used for test purposes, please ignore
BOB_ENABLE_VISUALISER="False"
BOB_OPTIMISED="True"
RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
FASTRTPS_DEFAULT_PROFILES_FILE="/workspaces/bobcamera/src/ros2/config/fastdds.xml"
```
### 11. To start up BOB
```
docker compose up
```
### 12. Use your system browser and navigate to [http://127.0.0.1:8080](http://127.0.0.1:8080)

### 13. To shut BOB down, type CTRL + C in the terminal