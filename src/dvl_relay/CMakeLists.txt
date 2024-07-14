cmake_minimum_required(VERSION 3.5)
project(dvl_relay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dvl_relay_node src/dvl_relay_node.cpp)
ament_target_dependencies(dvl_relay_node rclcpp geometry_msgs)

install(TARGETS
  dvl_relay_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
