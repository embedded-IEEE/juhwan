cmake_minimum_required(VERSION 3.8)
project(rc_nav2_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

include_directories(include)

add_library(rc_nav2_bt_nodes SHARED
  src/compute_approach_goal.cpp
  src/compute_approach_goals.cpp
  src/near_goal_stopped_condition.cpp
)

ament_target_dependencies(rc_nav2_bt_nodes
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
)

target_compile_definitions(rc_nav2_bt_nodes
  PRIVATE "RC_NAV2_BT_NODES_BUILDING_LIBRARY"
          "BT_PLUGIN_EXPORT")

install(TARGETS rc_nav2_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_tree/
  DESTINATION share/${PROJECT_NAME}/behavior_tree
)

ament_package()
