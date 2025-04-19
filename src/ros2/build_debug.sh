#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ros2_ws/install/setup.bash

mkdir -p cd ../boblib/build \
    && cd ../boblib/build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug .. \
    && cmake --build . -j$(nproc) \
    && sudo make install \
    && cd -

colcon build --executor parallel --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON