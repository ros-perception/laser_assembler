cmake_minimum_required(VERSION 3.5)
project(laser_assembler)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

set( filters_FOUND 1 )

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(laser_assembler_srv_gen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(filters REQUIRED)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS}
    ${rosidl_default_generators_INCLUDE_DIRS} ${signals_INCLUDE_DIRS}
    ${builtin_interfaces_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
    ${laser_geometry_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS}
    ${message_filters_INCLUDE_DIRS}
    ${laser_assembler_srv_gen_INCLUDE_DIRS}
    ${rostest_INCLUDE_DIRS} ${system_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${filters_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})

set(LIBS ${rclcpp_LIBRARIES} ${ament_cmake_LIBRARIES}
    ${rosidl_default_generators_LIBRARIES} ${signals_LIBRARIES}
    ${laser_assembler_srv_gen_LIBRARIES} ${std_msgs_LIBRARIES}
    ${system_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_LIBRARIES}
    ${laser_geometry_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${message_filters_LIBRARIES} ${sensor_msgs_LIBRARIES}
    ${filters_LIBRARIES})

add_executable(laser_scan_assembler src/laser_scan_assembler.cpp)
target_link_libraries(laser_scan_assembler ${LIBS})
install(
  TARGETS laser_scan_assembler
  DESTINATION lib/${PROJECT_NAME})

include_directories(include)

install(DIRECTORY "include/"
  DESTINATION include)

install(TARGETS laser_scan_assembler
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(signals)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2)
ament_export_dependencies(message_filters)
ament_export_dependencies(laser_geometry)
ament_export_include_directories(${INCLUDE_DIRS})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_executable(dummy_scan_producer test/dummy_scan_producer.cpp)
  target_link_libraries(dummy_scan_producer ${LIBS})
  install(
  TARGETS dummy_scan_producer
  DESTINATION lib/${PROJECT_NAME})

  ament_add_gtest(test_assembler test/test_assembler.cpp)
  target_link_libraries(test_assembler ${LIBS})
  install(
  TARGETS test_assembler
  DESTINATION lib/${PROJECT_NAME})

  add_executable(periodic_snapshotter examples/periodic_snapshotter.cpp)
  target_link_libraries(periodic_snapshotter ${LIBS})
  install(
  TARGETS periodic_snapshotter
  DESTINATION lib/${PROJECT_NAME})

endif()

ament_package()
