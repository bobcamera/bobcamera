cmake_minimum_required(VERSION 3.5)
project(bob_auxiliary)

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

# LifecycleServiceClient Node Library
add_library(information_node SHARED src/information.cpp)
target_compile_definitions(information_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(information_node PUBLIC "${cpp_typesupport_target}" boblib)
ament_target_dependencies(information_node PUBLIC rclcpp std_msgs rcl_interfaces rclcpp_components bob_interfaces)
rclcpp_components_register_nodes(information_node "Information")
install(TARGETS information_node DESTINATION lib)

ament_package()
