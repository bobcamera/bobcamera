cmake_minimum_required(VERSION 3.8)
project(bob_interfaces)

include_directories(/usr/local/lib/python3.10/dist-packages/numpy/core/include)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RecordingState.msg"
  "msg/RecordingEvent.msg"
  "msg/DetectorBBox.msg"
  "msg/DetectorBBoxArray.msg"
  "msg/DetectorState.msg"
  "msg/TrackingState.msg"
  "msg/MonitoringStatus.msg"
  "msg/ObserverCloudEstimation.msg"
  "msg/ObserverDayNight.msg"  
  "msg/TrackPoint.msg"
  "msg/TrackTrajectory.msg"
  "msg/TrackDetection.msg"
  "msg/Tracking.msg"
  "msg/LogMessage.msg"
  # "srv/BGSResetRequest.srv"
  "srv/ConfigEntryUpdate.srv"
  "srv/ApplicationInfo.srv"
  "srv/Mask.srv"
  "srv/MaskOverrideRequest.srv"
  "srv/MaskSvgDelete.srv"
  "srv/MaskSvgUpdate.srv"
  "srv/MaskGetSvg.srv"
  "srv/CameraSettings.srv"
  "srv/SensitivityChangeRequest.srv"
  "srv/RecordingRequest.srv"
  DEPENDENCIES std_msgs sensor_msgs vision_msgs geometry_msgs
)

ament_package()