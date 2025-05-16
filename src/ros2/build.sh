#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ros2_ws/install/setup.bash

mkdir -p cd ../boblib/build \
    && cd ../boblib/build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. \
    && cmake --build . -j$(nproc) \
    && sudo make install \
    && cd -

colcon build --executor parallel --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_PARALLEL_LEVEL=$(nproc) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
