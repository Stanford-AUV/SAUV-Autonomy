# Set minimum required CMake version and project name.
cmake_minimum_required(VERSION 3.5)
project(xsens_mti_ros2_driver)

# Set C++ standard, default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

link_directories(
  /usr/local/xsens/public/xspublic/xscontroller
  /usr/local/xsens/public/xspublic/xscommon
  /usr/local/xsens/public/xspublic/xstypes
  /usr/local/xsens/examples/mtsdk/xda_public_cpp/xspublic/xscontroller
  /usr/local/xsens/examples/mtsdk/xda_public_cpp/xspublic/xscommon
  /usr/local/xsens/examples/mtsdk/xda_public_cpp/xspublic/xstypes
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XsStatusWord.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
  /usr/local/xsens/include
  /usr/local/xsens/include/xstypes
  /usr/local/xsens/public/xspublic/xstypes
  /usr/local/xsens/xsens_ros_mti_driver/lib/xspublic/xstypes
  /usr/local/xsens/examples/mtsdk/xda_public_cpp/xspublic/xstypes
)

add_executable(
  xsens_mti_node
  src/main.cpp
  src/xdainterface.cpp
  src/xdacallback.cpp
  src/xsens_log_handler.cpp
  src/ntrip_util.cpp
  src/xsens_time_handler.cpp
)

add_dependencies(xsens_mti_node ${PROJECT_NAME})

ament_target_dependencies(xsens_mti_node 
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nmea_msgs
  mavros_msgs
  rosidl_default_runtime
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(xsens_mti_node "${cpp_typesupport_target}")

target_link_libraries(
  xsens_mti_node
  /usr/local/xsens/public/xspublic/xscontroller/libxscontroller.a
  /usr/local/xsens/public/xspublic/xscommon/libxscommon.a
  /usr/local/xsens/public/xspublic/xstypes/libxstypes.a
  pthread
  dl
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS xsens_mti_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY headers/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Export  ##
#############
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nmea_msgs
  mavros_msgs
  rosidl_default_runtime
)

ament_export_include_directories(
  include
)

ament_package()
