cmake_minimum_required(VERSION 3.5)
project(bob_recorder)

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
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(boblib REQUIRED)
find_package(bob_camera REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bob_interfaces REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../bob_shared/include")
include_directories(/usr/include/jsoncpp)
include_directories(${bob_camera_INCLUDE_DIRS})


if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wno-missing-braces)
ENDIF()

# # Video Recorder Node
# add_library(video_recorder SHARED src/video_recorder.cpp)
# target_compile_definitions(video_recorder PRIVATE "COMPOSITION_BUILDING_DLL")
# target_link_libraries(video_recorder PUBLIC boblib jsoncpp)
# ament_target_dependencies(video_recorder PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV rclcpp_components bob_interfaces bob_camera)
# rclcpp_components_register_nodes(video_recorder "VideoRecorder")
# install(TARGETS video_recorder DESTINATION lib)

# # json Recorder Node
# add_library(json_recorder SHARED src/json_recorder.cpp)
# target_compile_definitions(json_recorder PRIVATE "COMPOSITION_BUILDING_DLL")
# ament_target_dependencies(json_recorder PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters rclcpp_components bob_interfaces bob_camera)
# target_link_libraries(json_recorder PUBLIC jsoncpp)
# rclcpp_components_register_nodes(json_recorder "JsonRecorder")
# install(TARGETS json_recorder DESTINATION lib)

# Recorder Manager
add_library(record_manager SHARED src/record_manager.cpp)
target_compile_definitions(record_manager PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(record_manager PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV rclcpp_components bob_interfaces bob_camera)
target_link_libraries(record_manager PUBLIC jsoncpp)
rclcpp_components_register_nodes(record_manager "RecordManager")
install(TARGETS record_manager DESTINATION lib)

# Recording Monitor Node
add_library(recording_monitor_node SHARED src/recording_monitor.cpp)
target_compile_definitions(recording_monitor_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(recording_monitor_node PUBLIC boblib)
ament_target_dependencies(recording_monitor_node PUBLIC rclcpp sensor_msgs rclcpp_components bob_interfaces)
rclcpp_components_register_nodes(recording_monitor_node "RecordingMonitor")
install(TARGETS recording_monitor_node DESTINATION lib)

ament_package()
