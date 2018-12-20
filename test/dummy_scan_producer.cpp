// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Vijay Pradeep */

/**
 * Generate dummy scans in order to not be dependent on bag files in order to
 *run tests
 **/

#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


#define ROS_INFO printf

void runLoop(rclcpp::Node::SharedPtr node_)
{
  rclcpp::Rate loopRate(5);

  auto scan_pub =
    node_->create_publisher<sensor_msgs::msg::LaserScan>("dummy_scan", 100);

  // Configure the Transform broadcaster
  tf2_ros::TransformBroadcaster broadcaster(node_);
  tf2::Transform laser_transform(tf2::Quaternion(0, 0, 0, 1));

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

  for (unsigned int i = 0; i < N; i++) {
    scan.ranges[i] = 10.0;
    scan.intensities[i] = 10.0;
  }

  // Keep sending scans until the assembler is done
  while (rclcpp::ok()) {
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

int main(int argc, char ** argv)
{
  printf("dummy scan producer\n");
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
    rclcpp::Node::make_shared("dummy_scan_producer");
  std::thread run_thread(&runLoop, node);
  rclcpp::spin(node);
  run_thread.join();
  return 0;
}
