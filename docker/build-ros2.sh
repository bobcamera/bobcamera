#!/usr/bin/env bash
version=1.0.5
docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/bob-ros2-dev:$version -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev