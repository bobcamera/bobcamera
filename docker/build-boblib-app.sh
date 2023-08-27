#!/usr/bin/env bash
version=1.0.5
docker buildx build --load --platform linux/amd64 -f Dockerfile . -t bobcamera/boblib-app:$version -t bobcamera/boblib-app:latest --target boblib-app
#docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/boblib-app:$version -t bobcamera/boblib-app:latest --target boblib-app
