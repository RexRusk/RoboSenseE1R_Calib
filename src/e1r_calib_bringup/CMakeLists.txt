cmake_minimum_required(VERSION 3.8)
project(e1r_calib_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
