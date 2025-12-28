cmake_minimum_required(VERSION 3.10)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)         # optional, but common
find_package(ignition-transport11 REQUIRED)   # optional if you use gz transport

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)   # ✅ 추가

add_library(conveyor_plugin SHARED conveyor_plugin.cpp)
target_compile_features(conveyor_plugin PUBLIC cxx_std_17)

ament_target_dependencies(conveyor_plugin
  rclcpp
  std_msgs
  std_srvs                 # ✅ 추가
)

target_link_libraries(conveyor_plugin
  ignition-gazebo6::ignition-gazebo6
  ignition-math6::ignition-math6
  ignition-transport11::ignition-transport11
)

install(
  TARGETS conveyor_plugin
  LIBRARY DESTINATION lib
)