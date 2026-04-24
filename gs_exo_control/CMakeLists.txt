cmake_minimum_required(VERSION 3.22)
project(gs_exo_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(exo_controller_node
  src/exo_controller_node.cpp
  src/admittance_controller.cpp
  src/vesc_interface.cpp
)
ament_target_dependencies(exo_controller_node rclcpp gs_msgs geometry_msgs std_msgs)

install(TARGETS exo_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
