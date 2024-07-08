cmake_minimum_required(VERSION 3.8)
project(dvl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(PDDecoder
  src/PDDecoder/PDD_Decoder.c
  src/PDDecoder/PDD_Ensemble.c
  src/PDDecoder/PDD_Helper.c)

include_directories(
  include
  src/PDDecoder
)

add_executable(dvl_talker src/dvl_publisher.cpp)
ament_target_dependencies(dvl_talker rclcpp std_msgs geometry_msgs)

target_link_libraries(dvl_talker PDDecoder)

install(TARGETS
  dvl_talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
