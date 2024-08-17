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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(boblib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(bob_interfaces REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../bob_shared/include")
include_directories(/usr/include/jsoncpp)

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

# Camera BGS Node
add_library(camera_bgs_node SHARED src/camera_bgs.cpp)
target_compile_definitions(camera_bgs_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(camera_bgs_node PUBLIC jsoncpp boblib "${cpp_typesupport_target}")
ament_target_dependencies(camera_bgs_node PUBLIC
    rclcpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    rcl_interfaces 
    rclcpp_components
    rclcpp_lifecycle
    OpenCV
    Boost
    bob_interfaces
    lifecycle_msgs)
rclcpp_components_register_nodes(camera_bgs_node "CameraBGS")
install(TARGETS camera_bgs_node DESTINATION lib)

ament_package()
