cmake_minimum_required(VERSION 3.5)

# Package definition
project(sky360_camera)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(sky360lib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../sky360_shared/include")

set(msg_files
    "msg/AreaLimits.msg"
    "msg/BayerFormat.msg"
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

# All Sky Publisher Node
add_executable(all_sky_publisher_node src/all_sky_publisher.cpp)
target_link_libraries(all_sky_publisher_node PUBLIC sky360lib "${cpp_typesupport_target}")
ament_target_dependencies(all_sky_publisher_node PUBLIC
    rclcpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    rcl_interfaces 
    OpenCV
    Boost)
install(TARGETS all_sky_publisher_node DESTINATION lib/${PROJECT_NAME})

# Web Camera Publisher Node
add_executable(web_camera_publisher_node src/web_camera_publisher.cpp)
target_link_libraries(web_camera_publisher_node PUBLIC sky360lib "${cpp_typesupport_target}")
ament_target_dependencies(web_camera_publisher_node PUBLIC
    rclcpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    rcl_interfaces 
    OpenCV
    Boost)
install(TARGETS web_camera_publisher_node DESTINATION lib/${PROJECT_NAME})

ament_package()
