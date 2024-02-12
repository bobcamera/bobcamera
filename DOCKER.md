# Docker instructions etc

NOTE: DO NOT INSTALL RQT and be careful with ROS libraries that are dependent on an older version of OpenCVand 

BOB uses a multistage dockerfile to builds its containers.

### Run this ONE TIME
```
docker buildx create --name builder_mp --use --bootstrap
```

### If you would like to build the dev container used by vscode run the following:
- The "--progress plain" switch provides all the build output, if you are not interested in that feel free to omit this switch.
- The "--no-cache" switch will always rebuild the containers no matter is nothing has changed.

NOTE: The use of the progress plain and no-cache switches, if you don't need them then don't use them

#### *NOTE: For the life of me, I can't get this to work, no matter what I do, VsCode will pull the image from docker hub versus using the local image.*

#### Building to the **opencv** target and then exporting to docker locally
```
docker buildx build --progress plain --no-cache --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-opencv:1.0.1 -t bobcamera/bob-opencv:latest --target opencv --output=type=docker
```

#### Building to the **bob-ros2-iron-dev** target and then exporting to docker locally
```
docker buildx build --progress plain --no-cache --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-dev2:1.1.0 -t bobcamera/bob-ros2-iron-dev2:latest --target bob-ros2-iron-dev --output=type=docker
```

#### Building to the **bob-ros2-iron-prod** target and then exporting to docker locally
```
docker buildx build --progress plain --no-cache --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-prod:1.1.1 -t bobcamera/bob-ros2-iron-prod:latest --target bob-ros2-iron-prod --output=type=docker
```

### If you need to rebuild the docker images and push to docker hub, please use the following commands

NOTE: The use of the progress plain and no-cache switches, if you don't need them then don't use them

#### Building to the **opencv** target and then exporting to docker hub
```
docker buildx build --push --progress plain --no-cache --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-opencv:1.0.1 -t bobcamera/bob-opencv:latest --target opencv
```

#### Building to the **bob-ros2-iron-dev** target and then exporting to docker hub
```
docker buildx build --push --progress plain --no-cache --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-dev2:1.1.0 -t bobcamera/bob-ros2-iron-dev2:latest --target bob-ros2-iron-dev
```

#### Building to the **bob-ros2-iron-prod** target and then exporting to docker hub
```
docker buildx build --push --progress plain --no-cache --platform linux/amd64 -f Dockerfile . -t bobcamera/bob-ros2-iron-prod:1.1.1 -t bobcamera/bob-ros2-iron-prod:latest --target bob-ros2-iron-prod
```

<!-- START: this will not currently work 
### If you would like to rebuild the docker images used for docker compose, run the following:
- The "--no-cache" switch will always rebuild the containers no matter is nothing has changed.
```
docker build --no-cache -f Dockerfile --target bob-ros2-dev --progress plain .
```
END: this will not currently work -->

### Once you have build the **bob-ros2-iron-prod** target run the following docker compose compose command to complete the build container build locally.
- The "--progress plain" switch provides all the build output, if you are not interested in that feel free to omit this switch.
```
docker compose build --progress plain
```

### If you need to "reset" your docker environment i.e. remove everything run the following command
```
docker buildx prune && docker system prune -a -f --volumes 
```
##### Please NOTE the following WARNING!:
```
This will remove:
  - all stopped containers
  - all networks not used by at least one container
  - all volumes not used by at least one container
  - all images without at least one container associated to them
  - all build cache
```
### If you need to run a local container, please use one of the following commands:

```
docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/bob-opencv:latest bash
docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/bob-ros2-iron-dev2:latest bash
docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY bobcamera/bob-ros2-iron-prod:latest bash
```