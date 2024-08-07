#!/bin/bash

# Function to validate version number format
validate_version_number() {
    if [[ $1 =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        return 0  # Version number format is valid
    else
        return 1  # Version number format is not valid
    fi
}


web_version=$(grep -Po '(?<=image: bobcamera/bob-web-developer:)\d+\.\d+\.\d+' ./docker/docker-compose-developer.yaml)
bob_version=$(grep -Po '(?<=image: bobcamera/bob-ros2-developer:)\d+\.\d+\.\d+' ./docker/docker-compose-developer.yaml)
if [ "$web_version" == "$bob_version" ]; then
    echo "Docker compose version numbers are consistent: $web_version"
else
    echo "Docker compose version numbers are not consistent:"
    echo "Web Version: $web_version"
    echo "Bob Version: $bob_version"
    exit 1
fi


# Split version number into major, minor, and patch
IFS='.' read -r major minor patch <<< "$web_version"


while :; do 
    read -p "Select y(es) to increment the patch-number or n(o) to specify a version number." patchincrement
    response_lower=$(echo "$patchincrement" | tr '[:upper:]' '[:lower:]')
    if [ "$response_lower" = "no" ] || [ "$response_lower" = "n" ]; then
        # Prompt the user for a version number
        while :; do
            read -p "Enter version number (format: number.number.number): " version_number
            # Validate the version number format
            if validate_version_number "$version_number"; then
                echo ""
                echo "New version number: $version_number"
                break  # Exit the loop if the version number format is valid
            else
                echo "Invalid version number format. Please enter a version number in the format number.number.number"
            fi
        done
        break
    elif [ "$response_lower" = "yes" ] || [ "$response_lower" = "y" ]; then
         # Increment the patch version
        patch=$((patch + 1))
        version_number="$major.$minor.$patch"
        echo ""
        echo "New version number: $version_number"
        break
    else
        echo "Invalid response. Please enter '(y)es' or '(n)o'."
    fi
done

echo ""

# Replace version number in docker-compose-developer.yaml
if sed -i "s/bobcamera\/bob-web-developer:$web_version/bobcamera\/bob-web-developer:$version_number/" ./docker/docker-compose-developer.yaml && \
   sed -i "s/bobcamera\/bob-ros2-developer:$bob_version/bobcamera\/bob-ros2-developer:$version_number/" ./docker/docker-compose-developer.yaml; then
    echo "Version number successfully updated - ./docker/docker-compose-developer.yaml."
else
    echo "Failed to update version number - ./docker/docker-compose-developer.yaml."
fi

# Replace version number in app_config.yaml
if sed -i "s/application_version: '$web_version'/application_version: '$version_number'/" src/ros2/src/bob_launch/config/app_config.yaml; then
    echo "Version number successfully updated - launch/config/app_config.yaml."
else
    echo "Failed to update version number - launch/config/app_config.yaml."
fi

echo ""

read -rp "Press Enter to continue with building and pushing docker images."

rm -r ./src/ros2/assets/calibration/*
rm -r ./src/ros2/assets/config/*
rm -r ./src/ros2/assets/masks/*
rm -r ./src/ros2/assets/recordings/*
rm -r ./src/ros2/assets/wsdl/*

# docker build \
#     --progress=plain \
#     --push \
#     --platform linux/amd64 \
#     -f ./docker/Dockerfile . \
#     -t bobcamera/bob-opencv:$version_number \
#     -t bobcamera/bob-opencv:latest \
#     --target opencv

# docker build \
#     --progress=plain \
#     --platform linux/amd64 \
#     -f ./docker/Dockerfile . \
#     -t bobcamera/boblib:$version_number \
#     -t bobcamera/boblib:latest \
#     --target boblib

# docker build \
#     --progress=plain \
#     --platform linux/amd64 \
#     -f ./docker/Dockerfile . \
#     -t bobcamera/bob-boblibapp:$version_number \
#     -t bobcamera/bob-boblibapp:latest \
#     --target boblib-app

# docker build \
#     --progress=plain \
#     --push \
#     --platform linux/amd64 \
#     -f ./docker/Dockerfile . \
#     -t bobcamera/bob-ros2-dev:$version_number \
#     -t bobcamera/bob-ros2-dev:latest \
#     --target bob-ros2-dev

# docker build \
#     --progress=plain \
#     --platform linux/amd64 \
#     -f ./docker/Dockerfile . \
#     -t bobcamera/bob-ros2-build:$version_number \
#     -t bobcamera/bob-ros2-build:latest \
#     --target bob-ros2-build

docker build \
    --progress=plain \
    --push \
    --platform linux/amd64 \
    -f ./docker/Dockerfile . \
    -t bobcamera/bob-ros2-developer:$version_number \
    -t bobcamera/bob-ros2-developer:latest \
    --target bob-ros2-prod

docker build \
    --progress=plain \
    --push \
    --platform linux/amd64 \
    -f ./docker/Dockerfile . \
    -t bobcamera/bob-web-developer:$version_number \
    -t bobcamera/bob-web-developer:latest \
    --target bob-web-prod
