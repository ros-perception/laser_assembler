cmake_minimum_required(VERSION 3.5)
project(laser_assembler_srv_gen)

# Support C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/AssembleScans.srv"
    "srv/AssembleScans2.srv"
    DEPENDENCIES builtin_interfaces sensor_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
