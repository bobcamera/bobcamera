cmake_minimum_required(VERSION 3.5)
project(bob_image_processing)

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
find_package(jsoncpp REQUIRED)
find_package(bob_interfaces REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../bob_shared/include")
include_directories(/usr/include/jsoncpp)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wno-missing-braces)
ENDIF()

# Background Subtractor Node
add_library(background_subtractor_node SHARED src/background_subtractor.cpp)
target_compile_definitions(background_subtractor_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(background_subtractor_node PUBLIC jsoncpp boblib)
ament_target_dependencies(background_subtractor_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge OpenCV rclcpp_components bob_camera bob_interfaces)
rclcpp_components_register_nodes(background_subtractor_node "BackgroundSubtractor")
install(TARGETS background_subtractor_node DESTINATION lib)

# Background Subtractor v2 Node
add_library(background_subtractor2_node SHARED src/background_subtractor2.cpp)
target_compile_definitions(background_subtractor2_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(background_subtractor2_node PUBLIC jsoncpp boblib)
ament_target_dependencies(background_subtractor2_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge OpenCV rclcpp_components bob_camera bob_interfaces)
rclcpp_components_register_nodes(background_subtractor2_node "BackgroundSubtractor2")
install(TARGETS background_subtractor2_node DESTINATION lib)

# Annotated Frame Provider Node
add_library(annotated_frame_provider_node SHARED src/annotated_frame_provider.cpp)
target_compile_definitions(annotated_frame_provider_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(annotated_frame_provider_node PUBLIC boblib)
ament_target_dependencies(annotated_frame_provider_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV rclcpp_components bob_interfaces)
rclcpp_components_register_nodes(annotated_frame_provider_node "AnnotatedFrameProvider")
install(TARGETS annotated_frame_provider_node DESTINATION lib)

# Frame Resizer
add_library(frame_resizer SHARED src/frame_resizer.cpp)
target_compile_definitions(frame_resizer PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(frame_resizer PUBLIC boblib)
ament_target_dependencies(frame_resizer PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV rclcpp_components bob_interfaces)
rclcpp_components_register_nodes(frame_resizer "FrameResizer")
install(TARGETS frame_resizer DESTINATION lib)

# Frame Compressor
add_library(frame_compressor SHARED src/frame_compressor.cpp)
target_compile_definitions(frame_compressor PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(frame_compressor PUBLIC boblib)
ament_target_dependencies(frame_compressor PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV rclcpp_components bob_interfaces)
rclcpp_components_register_nodes(frame_compressor "FrameCompressor")
install(TARGETS frame_compressor DESTINATION lib)

# Mask Application Node
add_library(mask_application_node SHARED src/mask_application.cpp)
target_compile_definitions(mask_application_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(mask_application_node PUBLIC jsoncpp boblib)
ament_target_dependencies(mask_application_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge OpenCV rclcpp_components bob_camera bob_interfaces)
rclcpp_components_register_nodes(mask_application_node "MaskApplication")
install(TARGETS mask_application_node DESTINATION lib)

# Track Sensitivity Monitor Node
add_library(track_sensitivity_monitor SHARED src/track_sensitivity_monitor.cpp)
target_compile_definitions(track_sensitivity_monitor PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(track_sensitivity_monitor PUBLIC boblib)
ament_target_dependencies(track_sensitivity_monitor PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge OpenCV rclcpp_components bob_camera bob_interfaces)
rclcpp_components_register_nodes(track_sensitivity_monitor "TrackSensitivityMonitor")
install(TARGETS track_sensitivity_monitor DESTINATION lib)

ament_package()
