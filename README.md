# bobcamera

Install dependencies
```
sudo apt-get install git curl
```
clone the github repo
```
git clone --recursive https://github.com/bobcamera/bobcamera.git
```

if you need a specific branch then clone that branch
```
git clone --recursive --branch branch-name https://github.com/bobcamera/bobcamera.git
```
change directory to bobcamera
```
cd bobcamera
```
set the setup script as executable
```
chmod +x setup.sh
```
run the setup script, this will install docker and cocker compose
```
./setup.sh
```
reboot the machine
```
sudo shutdown -r now
```
create the .env file
```
touch .env
```
edit the .env file
```
nano .env
```
paste this into the .env file (it's a start, you will need to configure it correctly)
```
BOB_SOURCE="'rtsp'"
BOB_RTSP_URL="rtsp://bob:Sky360Sky!@10.20.30.58:554/Streaming/Channels/101"
BOB_RTSP_WIDTH="2560"
BOB_RTSP_HEIGHT="2560"
BOB_CAMERA_ID="0"
BOB_ENABLE_VISUALISER="False"
BOB_OPTIMISED="True"
BOB_ENABLE_RECORDING="False"
RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
FASTRTPS_DEFAULT_PROFILES_FILE="/workspaces/bobcamera/src/ros2/config/fastdds.xml"
BOB_BGS_ALGORITHM="vibe"
BOB_TRACKING_SENSITIVITY="'medium'"
BOB_TRACKING_USEMASK="False"
BOB_TRACKING_MASK_FILE="assets/masks/mask.jpg"
BOB_SIMULATION_WIDTH="2560"
BOB_SIMULATION_HEIGHT="2560"
BOB_SIMULATION_NUM_OBJECTS="5"
```
Bring up the docker images
```
docker compose up
```
use your system browser and browse to [http://127.0.0.1:8080](http://127.0.0.1:8080)
