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

#include <string>
#include <thread>
#include <chrono>
#include <memory>
#include "laser_assembler_srv_gen/srv/assemble_scans.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "gtest/gtest.h"

#define ROS_INFO printf

// static const string SERVICE_NAME = "assemble_scans";
static const char SERVICE_NAME[] = "assemble_scans";

class TestAssembler : public testing::Test
{
public:
  rclcpp::Node::SharedPtr node_;
  // ROS_INFO("Service [%s] detected", SERVICE_NAME.c_str());
  rclcpp::Client<laser_assembler_srv_gen::srv::AssembleScans>::SharedPtr
    client_;

  TestAssembler() {}

  static void spinThread(rclcpp::Node::SharedPtr node_) {rclcpp::spin(node_);}

  void SetUp()
  {
    node_ = rclcpp::Node::make_shared("test_assembler");
    std::thread spin_thread(spinThread, node_);
    spin_thread.detach();
    client_ = node_->create_client<laser_assembler_srv_gen::srv::AssembleScans>(
      "assemble_scans");
    using namespace std::chrono_literals;
    RCLCPP_INFO(node_->get_logger(), "Waiting for service [%s]", SERVICE_NAME);
    if (!client_->wait_for_service(20s)) {
      RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Service assemble_scans started successfully");
    received_something_ = false;
    got_cloud_ = false;

    auto scan_callback =
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) -> void {
        this->ScanCallback(msg);
      };
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "dummy_scan", scan_callback);
  }

  void ScanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!received_something_) {
      // Store the time of this first scan. Will be needed when we make the
      // service call
      start_time_ = scan_msg->header.stamp;
      received_something_ = true;
    } else {
      // Make the call to get a point cloud

      using namespace std::chrono_literals;
      std::this_thread::sleep_for(0.2s);
      auto request = std::make_shared<
        laser_assembler_srv_gen::srv::AssembleScans::Request>();
      request->begin = start_time_.sec;
      request->end = scan_msg->header.stamp.sec;

      // In order to wait for a response to arrive, we need to spin().
      // However, this function is already being called from within another
      // spin(). Unfortunately, the current version of spin() is not recursive
      // and so we cannot call spin() from within another spin(). Therefore, we
      // cannot wait for a response to the request we just made here within this
      // callback, because it was executed by some other spin function. The
      // workaround for this is to give the async_send_request() method another
      // argument which is a callback that gets executed once the future is
      // ready. We then return from this callback so that the existing spin()
      // function can continue and our callback will get called once the
      // response is received.
      using ServiceResponseFuture = rclcpp::Client<
        laser_assembler_srv_gen::srv::AssembleScans>::SharedFuture;

      auto response_received_callback = [this](ServiceResponseFuture future) {
          auto result = future.get();
          RCLCPP_INFO(node_->get_logger(), "Got result: [ %ld ]", result.get()->cloud.points.size());
          if (result.get()->cloud.points.size() > 0) {
            cloud_ = result.get()->cloud;
            got_cloud_ = true;
            cloud_condition_.notify_all();
          } else {
            RCLCPP_INFO(node_->get_logger(), "Got an empty cloud. Going to try again on the next scan");
          }
        };

      auto result =
        client_->async_send_request(request, response_received_callback);
    }
  }

protected:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  bool received_something_;

  builtin_interfaces::msg::Time start_time_;

  bool got_cloud_;
  sensor_msgs::msg::PointCloud cloud_;

  std::mutex mutex_;

  std::condition_variable cloud_condition_;
};

// Check to make sure we can get a point cloud with at least 1 point in it
TEST_F(TestAssembler, non_zero_cloud_test) {
  // Wait until we get laser scans
  std::unique_lock<std::mutex> lock(mutex_);

  while (rclcpp::ok() && !got_cloud_) {
    cloud_condition_.wait(lock);
  }

  EXPECT_LT((unsigned int)0, cloud_.points.size());

  SUCCEED();
}

int main(int argc, char ** argv)
{
  printf("******* Starting application *********\n");
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
