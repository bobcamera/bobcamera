cmake_minimum_required(VERSION 3.5)

# Package definition
project(bob_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
    find_package(TBB REQUIRED)  # for <execution> stdlib
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(boblib REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../bob_shared/include")

set(msg_files
    "msg/AreaLimits.msg"
    "msg/CameraInfo.msg"
    "msg/ChipInfo.msg"
    "msg/ImageInfo.msg"
    "msg/ParamLimits.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES builtin_interfaces std_msgs sensor_msgs)

ament_export_dependencies(rosidl_default_runtime)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wno-missing-braces)
ENDIF()

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# End package definition

# QhyNode
add_library(qhy_node SHARED src/qhy_node.cpp)
target_compile_definitions(qhy_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(qhy_node PUBLIC boblib "${cpp_typesupport_target}")
ament_target_dependencies(qhy_node PUBLIC
    rclcpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    rcl_interfaces 
    rclcpp_components
    OpenCV
    Boost)
rclcpp_components_register_nodes(qhy_node "QhyNode")
install(TARGETS qhy_node DESTINATION lib)

# Web Camera Video Node
add_library(web_camera_video_node SHARED src/web_camera_video.cpp)
target_compile_definitions(web_camera_video_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(web_camera_video_node PUBLIC boblib "${cpp_typesupport_target}")
ament_target_dependencies(web_camera_video_node PUBLIC
    rclcpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    rcl_interfaces 
    rclcpp_components
    OpenCV
    Boost)
rclcpp_components_register_nodes(web_camera_video_node "WebCameraVideo")
install(TARGETS web_camera_video_node DESTINATION lib)

# IP Camera Node
add_library(ipcamera_node SHARED src/ipcamera_node.cpp)
target_compile_definitions(ipcamera_node PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(ipcamera_node PUBLIC
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  sensor_msgs
  camera_info_manager
  camera_calibration_parsers
  image_transport)
rclcpp_components_register_nodes(ipcamera_node "IPCamera")
install(TARGETS ipcamera_node DESTINATION lib)

ament_package()
