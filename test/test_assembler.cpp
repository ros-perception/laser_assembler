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

#include <string>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_assembler_srv_gen/srv/assemble_scans.hpp"
#include <thread>

#define ROS_INFO printf

using namespace sensor_msgs;
using namespace std;

static const string SERVICE_NAME = "assemble_scans";

class TestAssembler : public testing::Test
{
public:

  rclcpp::Node::SharedPtr node;
  //ROS_INFO("Service [%s] detected", SERVICE_NAME.c_str());
  rclcpp::Client<laser_assembler_srv_gen::srv::AssembleScans>::SharedPtr client_;
  
  TestAssembler() {}
  
  static void spinThread(rclcpp::Node::SharedPtr node_)
  {
    rclcpp::spin(node_);
  }

  void SetUp()
  {
    //ROS_INFO("Waiting for service [%s]", SERVICE_NAME.c_str());
    node = rclcpp::Node::make_shared("test_assembler");
    std::thread spin_thread(spinThread, node);
    spin_thread.detach();
    client_ = node->create_client<laser_assembler_srv_gen::srv::AssembleScans>("assemble_scans");
    if (!client_->wait_for_service(20s)) {
      printf("Service not available after waiting");
      return;
    }
    printf(" Service assemble_scans started successfully");
    //ROS_INFO("Service [%s] detected", SERVICE_NAME.c_str());
    received_something_ = false;
    got_cloud_ = false;

    auto scan_callback =
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) -> void
    {
      this->ScanCallback(msg);
    };
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>("dummy_scan", scan_callback);
  }

  void ScanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
  {
    std::unique_lock<std::mutex> lock(mutex_);  
    if (!received_something_)
    {
      // Store the time of this first scan. Will be needed when we make the service call
      start_time_ = scan_msg->header.stamp;
      received_something_ = true;
    }
    else
    {
      // Make the call to get a point cloud
      
      std::this_thread::sleep_for(0.2s);
      auto request = std::make_shared<laser_assembler_srv_gen::srv::AssembleScans::Request>();
      request->begin = start_time_.sec;
      request->end = scan_msg->header.stamp.sec;
      
      // In order to wait for a response to arrive, we need to spin().
      // However, this function is already being called from within another spin().
      // Unfortunately, the current version of spin() is not recursive and so we
      // cannot call spin() from within another spin().
      // Therefore, we cannot wait for a response to the request we just made here
      // within this callback, because it was executed by some other spin function.
      // The workaround for this is to give the async_send_request() method another
      // argument which is a callback that gets executed once the future is ready.
      // We then return from this callback so that the existing spin() function can
      // continue and our callback will get called once the response is received.
      using ServiceResponseFuture =
        rclcpp::Client<laser_assembler_srv_gen::srv::AssembleScans>::SharedFuture;
      
      auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        printf("\nCloud points size = %ld\n", result.get()->cloud.points.size());
        //RCLCPP_INFO(this->get_logger(), "Got result: [%" PRId64 "]", future.get()->sum)
        if (result.get()->cloud.points.size() > 0 ) {
          
          cloud_ = result.get()->cloud;
          got_cloud_ = true;
          cloud_condition_.notify_all();
        }
        else
          ROS_INFO("Got an empty cloud. Going to try again on the next scan");
      };
      
      
      
      auto result = client_->async_send_request(request, response_received_callback);
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
TEST_F(TestAssembler, non_zero_cloud_test)
{
  // Wait until we get laser scans
  std::unique_lock<std::mutex> lock(mutex_);

  while(rclcpp::ok() && !got_cloud_)
  {
    cloud_condition_.wait(lock);
  }

  EXPECT_LT((unsigned int) 0, cloud_.points.size());

  SUCCEED();
}

int main(int argc, char** argv)
{
  printf("******* Starting application *********\n");

  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
