#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ros2_ws/install/setup.bash

# Set compiler flags to use AVX2 but not AVX512
export CFLAGS="-mno-avx512f"
export CXXFLAGS="-mno-avx512f"

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
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_C_FLAGS="$CFLAGS" \
    -DCMAKE_CXX_FLAGS="$CXXFLAGS"