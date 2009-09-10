cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_gensrv()

rosbuild_add_boost_directories()

rosbuild_add_executable(laser_scan_assembler_srv src/laser_scan_assembler_srv.cpp)
rosbuild_add_executable(point_cloud_assembler_srv src/point_cloud_assembler_srv.cpp)
rosbuild_add_executable(grab_cloud_data src/grab_cloud_data.cpp)

rosbuild_add_executable(merge_clouds src/merge_clouds.cpp)
rosbuild_link_boost(merge_clouds thread)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
