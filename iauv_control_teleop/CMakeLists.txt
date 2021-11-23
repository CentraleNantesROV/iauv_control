cmake_minimum_required(VERSION 3.5)
project(iauv_control_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS src/teleop.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
