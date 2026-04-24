cmake_minimum_required(VERSION 3.22)
project(gs_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticMap.msg"
  "msg/Traversability.msg"
  "msg/GuidanceTorque.msg"
  "msg/SafetyCmd.msg"
  "msg/MotorCmd.msg"
  "msg/GaitState.msg"
  "msg/Intent.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
