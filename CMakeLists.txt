cmake_minimum_required(VERSION 3.22)
project(r2d2_control)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY build/libs DESTINATION share/${PROJECT_NAME})
