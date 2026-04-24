cmake_minimum_required(VERSION 3.22)
project(gs_bringup)

if(NOT BUILD_TESTING)
  find_package(ament_cmake_python REQUIRED)
  ament_python_install_package(${PROJECT_NAME})
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
