cmake_minimum_required(VERSION 3.5)
project(iauv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(iauv_control_msgs REQUIRED)
find_package(uuv_gazebo_ros_plugins_msgs)
find_package(gazebo_msgs)

find_package(Eigen3 REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(IAUV_PLUGINS_MSGS "")
set(GAZEBO_MSGS "")

include_directories(include)

if(${uuv_gazebo_ros_plugins_msgs_FOUND})
    set(IAUV_PLUGINS_MSGS "uuv_gazebo_ros_plugins_msgs")
    add_definitions(-DWITH_IAUV_GAZEBO_PLUGINS)
endif()

# common lib, low-level control + model parsing
add_library(control_common SHARED src/common/cascaded_pid.cpp src/common/multi_cascaded_pid.cpp src/common/model_parser.cpp)
target_include_directories(control_common PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(control_common rclcpp urdf)

# base control node for interface
add_library(controller_io SHARED src/controller_io.cpp src/thruster_allocator.cpp)
target_link_libraries(controller_io control_common)
ament_target_dependencies(controller_io iauv_control_msgs rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros urdf ${IAUV_PLUGINS_MSGS})

# Body cascaded PID
add_executable(body_pid src/body_pid.cpp)
target_link_libraries(body_pid control_common controller_io)

# Body sliding mode
add_executable(body_sm src/body_sm.cpp)
target_link_libraries(body_sm control_common controller_io)

# Joint PID for Gazebo
if(${gazebo_msgs_FOUND})
    add_executable(joint_gz_pid src/joint_gz_pid.cpp)
    target_link_libraries(joint_gz_pid control_common)
    ament_target_dependencies(joint_gz_pid rclcpp sensor_msgs gazebo_msgs urdf)    
    install(TARGETS joint_gz_pid DESTINATION share/${PROJECT_NAME})
endif()

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

# auto-executable from node
#rclcpp_components_register_node(simulator_node
#  PLUGIN "map_simulator::SimulatorNode"
#  EXECUTABLE simulator)

install(TARGETS controller_io control_common body_pid body_sm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION share/${PROJECT_NAME})
  
install(PROGRAMS src/scripts/to_fixed_joints.py
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
