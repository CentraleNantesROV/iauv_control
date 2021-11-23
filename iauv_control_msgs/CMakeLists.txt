cmake_minimum_required(VERSION 3.5)
project(iauv_control_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Control.srv")
  
ament_package()
