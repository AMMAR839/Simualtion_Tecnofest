cmake_minimum_required(VERSION 3.8)
project(gamantaray_nav2_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(gamantaray_asv_obstacle_window_condition_bt_node SHARED
  src/asv_obstacle_window_condition.cpp
)

ament_target_dependencies(gamantaray_asv_obstacle_window_condition_bt_node
  behaviortree_cpp
  nav2_behavior_tree
  rclcpp
  sensor_msgs
)

target_compile_features(gamantaray_asv_obstacle_window_condition_bt_node
  PUBLIC cxx_std_17
)

target_compile_definitions(gamantaray_asv_obstacle_window_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT
)

install(TARGETS gamantaray_asv_obstacle_window_condition_bt_node
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

ament_export_include_directories(include)
ament_export_libraries(gamantaray_asv_obstacle_window_condition_bt_node)
ament_export_dependencies(
  behaviortree_cpp
  nav2_behavior_tree
  rclcpp
  sensor_msgs
)

ament_package()
