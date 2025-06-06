cmake_minimum_required(VERSION 3.5)
project(bob_visualizers)

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
find_package(boblib REQUIRED)
find_package(bob_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../bob_shared/include")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wno-missing-braces)
ENDIF()

# Frame Viewer Node
add_library(frame_viewer_node SHARED src/frame_viewer.cpp)
target_compile_definitions(frame_viewer_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(frame_viewer_node PUBLIC boblib)
ament_target_dependencies(frame_viewer_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV bob_interfaces rclcpp_components)
rclcpp_components_register_nodes(frame_viewer_node "FrameViewer")
install(TARGETS frame_viewer_node DESTINATION lib)

# Frame Viewer Blobs Node
add_library(frame_viewer_blobs_node SHARED src/frame_viewer_blobs.cpp)
target_compile_definitions(frame_viewer_blobs_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(frame_viewer_blobs_node PUBLIC boblib)
ament_target_dependencies(frame_viewer_blobs_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV bob_interfaces rclcpp_components)
rclcpp_components_register_nodes(frame_viewer_blobs_node "FrameViewerBlobs")
install(TARGETS frame_viewer_blobs_node DESTINATION lib)

ament_package()
