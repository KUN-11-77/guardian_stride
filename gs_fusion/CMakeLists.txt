cmake_minimum_required(VERSION 3.22)
project(gs_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gs_msgs REQUIRED)

# 尝试查找 mraa 和 libgpiod（可选）
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(MRAA mraa)
  pkg_check_modules(GPIOD libgpiod)
endif()

# imu_node - 需要 mraa
add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs)
if(MRAA_FOUND)
  target_include_directories(imu_node PRIVATE ${MRAA_INCLUDE_DIRS})
  target_link_libraries(imu_node ${MRAA_LIBRARIES})
else()
  target_compile_definitions(imu_node PRIVATE NO_MRAA)
endif()

include_directories(include)

# tof_node - 需要 libgpiod
add_executable(tof_node src/tof_node.cpp src/gait_detector.cpp)
ament_target_dependencies(tof_node rclcpp sensor_msgs gs_msgs)
target_include_directories(tof_node PRIVATE ${CMAKE_SOURCE_DIR}/include)
if(GPIOD_FOUND)
  target_include_directories(tof_node PRIVATE ${GPIOD_INCLUDE_DIRS})
  target_link_libraries(tof_node ${GPIOD_LIBRARIES})
else()
  target_compile_definitions(tof_node PRIVATE NO_GPIOD)
endif()

# occupancy_bridge - 不需要额外依赖
add_executable(occupancy_bridge src/occupancy_bridge.cpp)
ament_target_dependencies(occupancy_bridge rclcpp nav_msgs sensor_msgs tf2_ros)

install(TARGETS imu_node tof_node occupancy_bridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
