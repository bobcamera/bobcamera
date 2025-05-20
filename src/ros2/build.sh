#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ros2_ws/install/setup.bash

mkdir -p ../boblib/build \
  && cd ../boblib/build \
  && cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. \
  && cmake --build . --parallel "$(nproc)" \
  && sudo cmake --build . --target install \
  && cd -

# build ROS2 workspace
colcon build \
  --executor parallel \
  --parallel-workers "$(nproc)" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON