/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Vijay Pradeep */

/**
 * Generate dummy scans in order to not be dependent on bag files in order to run tests
 **/

#include<iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "tf2/LinearMath/Transform.h"


#define ROS_INFO printf

void runLoop(rclcpp::Node::SharedPtr node_)
{
  rclcpp::Rate loopRate(5);
  
  auto scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>("dummy_scan", 100);

  // Configure the Transform broadcaster
  tf2_ros::TransformBroadcaster broadcaster(node_);
  tf2::Transform laser_transform(tf2::Quaternion(0,0,0,1));

  // Populate the dummy laser scan
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "dummy_laser_link";
  scan.angle_min = 0.0;
  scan.angle_max = 99.0;
  scan.angle_increment = 1.0;
  scan.time_increment = .001;
  scan.scan_time = .05;
  scan.range_min = .01;
  scan.range_max = 100.0;

  const unsigned int N = 100;
  scan.ranges.resize(N);
  scan.intensities.resize(N);

  for (unsigned int i=0; i<N; i++)
  {
    scan.ranges[i] = 10.0;
    scan.intensities[i] = 10.0;
  }

  // Keep sending scans until the assembler is done
  while(rclcpp::ok())
  {
    scan.header.stamp = rclcpp::Clock().now();
    ROS_INFO("Publishing scan\n");
    scan_pub->publish(scan);
      
    geometry_msgs::msg::TransformStamped tf_transform;
    tf_transform.header.stamp = rclcpp::Clock().now();
    tf_transform.header.frame_id = "dummy_laser_link";
    tf_transform.child_frame_id = "dummy_base_link";
    tf_transform.transform.rotation.w = 1.0;
    broadcaster.sendTransform(tf_transform);
    loopRate.sleep();
    ROS_INFO("Publishing scan");
  }
}

int main(int argc, char** argv)
{
  printf("dummy scan producer\n");
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dummy_scan_producer");
  std::thread run_thread(&runLoop, node);
  rclcpp::spin(node);
  run_thread.join();
  return 0;
}
