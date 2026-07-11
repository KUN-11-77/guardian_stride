cmake_minimum_required(VERSION 3.22)
project(guardian_voice)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT BUILD_TESTING)
  find_package(ament_cmake_python REQUIRED)
  ament_python_install_package(${PROJECT_NAME})
endif()

install(DIRECTORY launch config resource DESTINATION share/${PROJECT_NAME})

ament_package()
