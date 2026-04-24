cmake_minimum_required(VERSION 3.22)
project(gs_safety)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gs_msgs REQUIRED)

include_directories(include)

# ========== collision_checker ==========
add_executable(collision_checker src/collision_checker.cpp)

ament_target_dependencies(collision_checker
  rclcpp std_msgs geometry_msgs sensor_msgs
)

target_link_libraries(collision_checker
  dl pthread
)

# ========== emergency_stop ==========
add_executable(emergency_stop src/emergency_stop.cpp)

ament_target_dependencies(emergency_stop
  rclcpp std_msgs gs_msgs
)

target_link_libraries(emergency_stop
  dl pthread
)

# ========== safety_monitor ==========
add_executable(safety_monitor src/safety_monitor.cpp)

ament_target_dependencies(safety_monitor
  rclcpp std_msgs geometry_msgs gs_msgs
)

target_link_libraries(safety_monitor
  dl pthread
)

install(TARGETS collision_checker emergency_stop safety_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()