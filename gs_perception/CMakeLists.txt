cmake_minimum_required(VERSION 3.22)
project(gs_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# OpenVINO 路径配置：从环境变量 OPENVINO_ROOT 获取，或尝试标准位置
set(OPENVINO_ROOT "$ENV{OPENVINO_ROOT}" CACHE PATH "OpenVINO installation root")
if(NOT OPENVINO_ROOT AND EXISTS /opt/intel/openvino)
  set(OPENVINO_ROOT /opt/intel/openvino)
endif()

# 尝试添加 OpenVINO CMake 路径
if(OPENVINO_ROOT AND EXISTS ${OPENVINO_ROOT}/cmake)
  list(APPEND CMAKE_PREFIX_PATH ${OPENVINO_ROOT}/cmake)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 使用 pkg-config 查找 OpenCV（跨平台）
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(OPENCV REQUIRED opencv4)
endif()

# 如果 pkg-config 找不到，尝试 CMake 的 FindOpenCV
if(NOT OPENCV_FOUND)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
endif()

find_package(gs_msgs REQUIRED)

# 尝试查找 OpenVINO（如果找不到则跳过 segformer_node）
find_package(OpenVINO QUIET COMPONENTS Runtime)
set(HAVE_OPENVINO ${OpenVINO_FOUND})

include_directories(include)

# OpenCV 链接库
set(OPENCV_LIBS
    ${OPENCV_LIBRARIES}
    dl
    pthread
)

# ========== segformer_node（需要 OpenVINO）==========
if(HAVE_OPENVINO)
  # 添加 OpenVINO include 路径
  if(OPENVINO_ROOT)
    list(APPEND OpenVINO_INCLUDE_PATHS
      ${OPENVINO_ROOT}/runtime/include
    )
  endif()

  add_executable(segformer_node
    src/segformer_node.cpp
    src/semantic_visualizer.cpp
    src/traversability_calculator.cpp
  )

  target_include_directories(segformer_node PRIVATE ${OpenVINO_INCLUDE_PATHS})
  ament_target_dependencies(segformer_node
    rclcpp sensor_msgs std_msgs geometry_msgs
    cv_bridge gs_msgs
  )
  target_link_libraries(segformer_node ${OPENCV_LIBS} openvino::runtime)

  install(TARGETS segformer_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "OpenVINO not found. Skipping segformer_node (requires OpenVINO).")
endif()

# ========== costmap_bridge（不需要 OpenVINO）==========
add_executable(costmap_bridge src/costmap_bridge.cpp)

ament_target_dependencies(costmap_bridge
  rclcpp std_msgs cv_bridge
)

target_link_libraries(costmap_bridge ${OPENCV_LIBS})

install(TARGETS costmap_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models scripts DESTINATION share/${PROJECT_NAME})

ament_package()

# 打印状态
if(HAVE_OPENVINO)
  message(STATUS "OpenVINO enabled - segformer_node will be built")
else()
  message(STATUS "OpenVINO not found - segformer_node skipped, only costmap_bridge will be built")
endif()