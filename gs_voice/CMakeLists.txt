cmake_minimum_required(VERSION 3.22)
project(gs_voice)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gs_msgs REQUIRED)

# OpenVINO 是可选的（用于 NPU 推理）
find_package(OpenVINO QUIET)
if(OpenVINO_FOUND)
  message(STATUS "OpenVINO found, enabling NPU inference")
  set(HAVE_OPEN_VINO 1)
else()
  message(STATUS "OpenVINO not found, running in simulation mode")
  set(HAVE_OPEN_VINO 0)
endif()

add_executable(voice_node
  src/voice_node.cpp
  src/intent_parser.cpp
  src/tts_manager.cpp
)
ament_target_dependencies(voice_node rclcpp std_msgs gs_msgs)

if(OpenVINO_FOUND)
  target_link_libraries(voice_node openvino::runtime)
  target_compile_definitions(voice_node PRIVATE HAVE_OPEN_VINO=1)
else()
  target_compile_definitions(voice_node PRIVATE HAVE_OPEN_VINO=0)
endif()

install(TARGETS voice_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

ament_package()
