# Docker instructions etc

BOB uses a multistage dockerfile to builds its containers.

### Run this ONE TIME
```
docker buildx create --name builder_mp --use --bootstrap
```

### If you would like to build the dev container used by vscode run the following:
- The "--progress plain" switch provides all the build output, if you are not interested in that feel free to omit this switch.
- The "--no-cache" switch will always rebuild the containers no matter is nothing has changed.
```
<!-- docker build -f Dockerfile . -t bobcamera/ros2-dev:1.0.5 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev --progress plain --no-cache -->
docker buildx build --load --platform linux/amd64,linux/arm64 -f Dockerfile . -t bobcamera/ros2-dev:1.0.5 -t bobcamera/bob-ros2-dev:latest --target bob-ros2-dev --progress plain --no-cache
```

<!-- START: this will not currently work 
### If you would like to rebuild the docker images used for docker compose, run the following:
- The "--no-cache" switch will always rebuild the containers no matter is nothing has changed.
```
docker build --no-cache -f Dockerfile --target bob-ros2-dev --progress plain .
```
END: this will not currently work -->

### Once that is complete run the following docker compose compose command to complete the build container build locally.
- The "--progress plain" switch provides all the build output, if you are not interested in that feel free to omit this switch.
```
docker compose build --progress plain
```

### If you need to "reset" your docker environment i.e. remove everything run the following command
```
docker system prune -a -f --volumes 
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
