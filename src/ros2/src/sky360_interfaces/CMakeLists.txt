cmake_minimum_required(VERSION 3.8)
project(sky360_interfaces)

include_directories(/usr/local/lib/python3.10/dist-packages/numpy/core/include)

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
  #"msg/ConfigEntryUpdated.msg"
  "msg/ConfigEntryUpdatedArray.msg"
  #"msg/ClassifiedObject.msg"
  #"msg/ClassifiedObjectsArray.msg"
  "msg/ConfigItem.msg"
  "msg/ObserverCloudEstimation.msg"
  "msg/ObserverDayNight.msg"
  "msg/TrackingState.msg"
  "msg/TrackPoint.msg"
  "msg/TrackTrajectory.msg"
  "msg/TrackTrajectoryArray.msg"
  "msg/TrackDetection.msg"
  "msg/TrackDetectionArray.msg"
  "srv/ConfigEntryUpdate.srv"
  "srv/ConfigEntry.srv"
  "srv/ConfigEntryArray.srv"
  "srv/Mask.srv"
  "srv/MaskUpdate.srv"
  DEPENDENCIES std_msgs sensor_msgs vision_msgs geometry_msgs
)

ament_package()