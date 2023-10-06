cmake_minimum_required(VERSION 3.5)

# Package definition
project(bob_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../bob_shared/include")

ament_export_dependencies(rosidl_default_runtime)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wno-missing-braces)
ENDIF()

# Object Simulator Component Node and CircleFrameGenerator
add_library(moving_objects_simulation_node SHARED
    src/circle_frame_generator.cpp
    src/object_simulator_node.cpp
)
target_compile_definitions(moving_objects_simulation_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(moving_objects_simulation_node ${OpenCV_LIBRARIES})
ament_target_dependencies(moving_objects_simulation_node 
    rclcpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    rcl_interfaces 
    rclcpp_components
    OpenCV)
rclcpp_components_register_nodes(moving_objects_simulation_node PLUGIN "MovingObjectsSimulation")
install(TARGETS moving_objects_simulation_node DESTINATION lib)

# SimulationOverlayProviderNode 
add_library(simulation_overlay_provider_node SHARED
    src/simulation_overlay_provider_node.cpp
)
target_compile_definitions(simulation_overlay_provider_node PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(simulation_overlay_provider_node ${OpenCV_LIBRARIES})
ament_target_dependencies(simulation_overlay_provider_node 
    rclcpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    rcl_interfaces 
    rclcpp_components
    OpenCV)
rclcpp_components_register_nodes(simulation_overlay_provider_node PLUGIN "SimulationOverlayProviderNode")   
install(TARGETS simulation_overlay_provider_node DESTINATION lib/${PROJECT_NAME})

ament_package()
