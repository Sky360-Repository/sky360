cmake_minimum_required(VERSION 3.5)
project(sky360_visualizers)

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
find_package(sky360lib REQUIRED)
find_package(sky360_interfaces REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../sky360_shared/include")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wno-missing-braces)
ENDIF()

# Annotated Frame Provider Node
add_executable(annotated_frame_provider_node src/annotated_frame_provider.cpp)
target_link_libraries(annotated_frame_provider_node PUBLIC sky360lib)
ament_target_dependencies(annotated_frame_provider_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV sky360_interfaces)
install(TARGETS annotated_frame_provider_node DESTINATION lib/${PROJECT_NAME})

# Frame Viewer Node
add_executable(frame_viewer_node src/frame_viewer.cpp)
target_link_libraries(frame_viewer_node PUBLIC sky360lib)
ament_target_dependencies(frame_viewer_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV sky360_interfaces)
install(TARGETS frame_viewer_node DESTINATION lib/${PROJECT_NAME})

# Frame BBox Viewer Node
add_executable(frame_bbox_viewer_node src/frame_bbox_viewer.cpp)
target_link_libraries(frame_bbox_viewer_node PUBLIC sky360lib)
ament_target_dependencies(frame_bbox_viewer_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV sky360_interfaces)
install(TARGETS frame_bbox_viewer_node DESTINATION lib/${PROJECT_NAME})

ament_package()
