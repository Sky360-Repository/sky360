cmake_minimum_required(VERSION 3.5)
project(sky360_monitoring)

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
find_package(sky360lib REQUIRED)
find_package(sky360_camera REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../sky360_shared/include")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wno-missing-braces)
ENDIF()

# Frame Saver Node
add_executable(frame_saver_node src/frame_saver.cpp)
target_link_libraries(frame_saver_node PUBLIC sky360lib)
ament_target_dependencies(frame_saver_node PUBLIC rclcpp sensor_msgs cv_bridge OpenCV sky360_camera)
install(TARGETS frame_saver_node DESTINATION lib/${PROJECT_NAME})

ament_package()
