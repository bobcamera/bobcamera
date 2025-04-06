cmake_minimum_required(VERSION 3.5)
project(bob_tracking)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
    find_package(TBB REQUIRED)  # for <execution> stdlib
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(boblib REQUIRED)
find_package(bob_interfaces REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
    "../bob_shared/include"
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wno-missing-braces)
ENDIF()

# Tracking Node
add_library(track_provider_node SHARED src/track_provider.cpp src/sort/kalman_filter.cpp src/sort/sort_tracker.cpp src/sort/track.cpp)
target_compile_definitions(track_provider_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(track_provider_node PUBLIC boblib)
ament_target_dependencies(track_provider_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV rclcpp_components bob_interfaces)
rclcpp_components_register_nodes(track_provider_node "TrackProvider")
install(TARGETS track_provider_node DESTINATION lib)

ament_package()
