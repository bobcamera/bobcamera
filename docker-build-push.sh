#!/bin/bash

# Function to validate version number format
validate_version_number() {
    if [[ $1 =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        return 0  # Version number format is valid
    else
        return 1  # Version number format is not valid
    fi
}

# Prompt the user for a version number
while :; do
    read -p "Enter version number (format: number.number.number): " version_number

    # Validate the version number format
    if validate_version_number "$version_number"; then
        break  # Exit the loop if the version number format is valid
    else
        echo "Invalid version number format. Please enter a version number in the format number.number.number"
    fi
done

# If the version number format is correct, continue with the script
echo "Version number entered: $version_number"

rm -r ./src/ros2/assets/calibration/*
rm -r ./src/ros2/assets/config/*
rm -r ./src/ros2/assets/masks/*
rm -r ./src/ros2/assets/recording/*
rm -r ./src/ros2/assets/wsdl/*

docker build \
    --progress=plain \
    --platform linux/amd64 \
    -f Dockerfile . \
    -t bobcamera/bob-opencv:$version_number \
    -t bobcamera/bob-opencv:latest \
    --target opencv

docker build \
    --progress=plain \
    --platform linux/amd64 \
    -f Dockerfile . \
    -t bobcamera/boblib:$version_number \
    -t bobcamera/boblib:latest \
    --target boblib

docker build \
    --progress=plain \
    --platform linux/amd64 \
    -f Dockerfile . \
    -t bobcamera/bob-boblibapp:$version_number \
    -t bobcamera/bob-boblibapp:latest \
    --target boblib-app

docker build \
    --progress=plain \
    --push \
    --platform linux/amd64 \
    -f Dockerfile . \
    -t bobcamera/bob-ros2-iron-dev:$version_number \
    -t bobcamera/bob-ros2-iron-dev:latest \
    --target bob-ros2-iron-dev

docker build \
    --progress=plain \
    --platform linux/amd64 \
    -f Dockerfile . \
    -t bobcamera/bob-ros2-iron-build:$version_number \
    -t bobcamera/bob-ros2-iron-build:latest \
    --target bob-ros2-iron-build

docker build \
    --progress=plain \
    --push \
    --platform linux/amd64 \
    -f Dockerfile . \
    -t bobcamera/bob-ros2-iron-prod:$version_number \
    -t bobcamera/bob-ros2-iron-prod:latest \
    --target bob-ros2-iron-prod

docker build \
    --progress=plain \
    --push \
    --platform linux/amd64 \
    -f Dockerfile . \
    -t bobcamera/bob-web-prod:$version_number \
    -t bobcamera/bob-web-prod:latest \
    --target bob-web-prod