# Using the ROS 2 base image
FROM bobcamera/bob-ros2-dev:latest

# Any additional user arguments (like USERNAME from the provided Dockerfile)
ARG USERNAME=ros

# Installing necessary packages for rosbridge
RUN apt-get update && apt-get install -y \
    ros-iron-rosbridge-server \
    ros-iron-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

# Expose the default rosbridge port
EXPOSE 9090

# Set the entry point or CMD for the rosbridge server
CMD ["ros2", "run", "rosbridge_server", "rosbridge_websocket"]