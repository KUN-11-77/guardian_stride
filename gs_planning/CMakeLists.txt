cmake_minimum_required(VERSION 3.22)
project(gs_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Nav2 代价图语义插件（作为 pluginlib 插件编译为 shared library）
add_library(semantic_costmap_plugin SHARED
  src/semantic_costmap_plugin.cpp
)
ament_target_dependencies(semantic_costmap_plugin
  rclcpp nav2_costmap_2d pluginlib gs_msgs
)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

# 引导力矩计算节点
add_executable(guidance_torque_node
  src/guidance_torque_node.cpp
)
ament_target_dependencies(guidance_torque_node rclcpp gs_msgs geometry_msgs nav_msgs)

# Intent → Nav2 Goal 桥接节点
add_executable(intent_to_goal
  src/intent_to_goal.cpp
)
ament_target_dependencies(intent_to_goal rclcpp gs_msgs geometry_msgs nav2_msgs)

install(TARGETS semantic_costmap_plugin guidance_torque_node intent_to_goal
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})

ament_package()
