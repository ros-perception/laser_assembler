cmake_minimum_required(VERSION 2.4.6)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_gensrv()

rosbuild_add_boost_directories()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

rosbuild_add_executable(laser_scan_assembler_srv src/laser_scan_assembler_srv.cpp)
rosbuild_link_boost(laser_scan_assembler_srv system signals)
rosbuild_add_executable(point_cloud_assembler_srv src/point_cloud_assembler_srv.cpp)
rosbuild_link_boost(point_cloud_assembler_srv signals)
rosbuild_add_executable(laser_scan_assembler src/laser_scan_assembler.cpp)
rosbuild_link_boost(laser_scan_assembler system signals)
rosbuild_add_executable(point_cloud_assembler src/point_cloud_assembler.cpp)
rosbuild_link_boost(point_cloud_assembler signals)

rosbuild_add_executable(../test/dummy_scan_producer test/dummy_scan_producer.cpp)

rosbuild_add_executable(merge_clouds src/merge_clouds.cpp)
rosbuild_link_boost(merge_clouds thread signals)

rosbuild_add_executable(../test/test_assembler test/test_assembler.cpp)
rosbuild_add_gtest_build_flags(../test/test_assembler)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_laser_assembler.launch)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_subdirectory(examples)
