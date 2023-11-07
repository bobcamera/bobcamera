#!/bin/sh

set -o errexit
set -o nounset
IFS=$(printf '\n\t')

# Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
printf '\nDocker installed successfully\n\n'

printf 'Waiting for Docker to start...\n\n'
sleep 5

printf 'Adding user to the docker group...\n\n'
sudo usermod -aG docker $USER

printf 'Installing Docker Compose...\n\n'

# Docker Compose
mkdir -p ~/.docker/cli-plugins/
COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
curl -SL https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-linux-x86_64 -o ~/.docker/cli-plugins/docker-compose
chmod +x ~/.docker/cli-plugins/docker-compose
printf '\nDocker Compose installed successfully\n\n'

docker compose version

#UID=${UID} GID=${GID} docker-compose build
#UID=${UID} GID=${GID} docker-compose up

if test -f .env; then
  printf 'Deleting .env file in order to reseed...\n\n'
  rm .env
fi

echo "# Source options are \"'rtsp'\", \"'usb'\", \"'video'\", \"'simulate'\", \"'rtsp_overlay'\", \"'video_overlay'\"" >> .env
echo "BOB_SOURCE=\"'rtsp'\"" >> .env
echo "# URL to the rtsp main video stream" >> .env
echo "# If you are having trouble, maybe use something like VLC media player to ensure the rtsp url is correct" >> .env
echo "BOB_RTSP_URL=\"rtsp://username:password@10.20.30.58:554/Streaming/Channels/101\"" >> .env
echo "# What the rtsp image width is set to" >> .env
echo "BOB_RTSP_WIDTH=\"2560\"" >> .env
echo "# What the rtsp image height is set to" >> .env
echo "BOB_RTSP_HEIGHT=\"2560\"" >> .env
echo "# If you want to use a USB camera, then this is the camera index associated with the camera" >> .env
echo "BOB_CAMERA_ID=\"0\"" >> .env
echo "# Do you want to capture recordings i.e. video files of the objects that are being tracked?" >> .env
echo "BOB_ENABLE_RECORDING=\"False\"" >> .env
echo "# Back Ground Subtraction algorithm options are \"vibe\", \"wmv\"" >> .env
echo "BOB_BGS_ALGORITHM=\"vibe\"" >> .env
echo "# We have configured a pre set of tracking sensitivity options, as with everything, these are still work in progress" >> .env
echo "# Sensitivity options are \"'minimal'\", \"'low'\", \"'medium'\", \"'high'\"" >> .env
echo "BOB_TRACKING_SENSITIVITY=\"'medium'\"" >> .env
echo "# Mask configuration" >> .env
echo "# This is used for both privacy and motion detection purposes. We encourage you to define a mask as it will significantly reduce noise" >> .env
echo "BOB_TRACKING_USEMASK=\"False\"" >> .env
echo "BOB_TRACKING_MASK_FILE=\"assets/masks/mask.jpg\"" >> .env
echo "# If you would like to overlay simulation objects over the rtsp stream then you will need to set these" >> .env
echo "# The source also needs to be set to rtsp_overlay fro this to work" >> .env
echo "# Simulation section, the width and height need to match that of the frame source" >> .env
echo "BOB_SIMULATION_WIDTH=\"2560\"" >> .env
echo "BOB_SIMULATION_HEIGHT=\"2560\"" >> .env
echo "BOB_SIMULATION_NUM_OBJECTS=\"5\"" >> .env
echo "# This is mainly used for test purposes, please ignore" >> .env
echo "BOB_ENABLE_VISUALISER=\"False\"" >> .env
echo "BOB_OPTIMISED=\"True\"" >> .env
echo "RMW_IMPLEMENTATION=\"rmw_fastrtps_cpp\"" >> .env
echo "FASTRTPS_DEFAULT_PROFILES_FILE=\"/workspaces/bobcamera/src/ros2/config/fastdds.xml\"" >> .env

printf 'Completed reseed of .env file...\n\n'