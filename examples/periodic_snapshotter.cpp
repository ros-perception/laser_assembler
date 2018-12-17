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

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <chrono>
// Services
#include "laser_assembler_srv_gen/srv/assemble_scans.hpp"

// Messages
#include "sensor_msgs/msg/point_cloud.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/string.hpp"

#define ROS_ERROR printf
#define ROS_INFO printf
#define ROS_WARN printf
#define ROS_DEBUG printf

using namespace std::chrono_literals;
using namespace std::placeholders;
/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */
namespace laser_assembler
{

class PeriodicSnapshotter: public rclcpp::Node
{

public:

  explicit PeriodicSnapshotter(const std::string & service_name)
  : Node(service_name)
  {
    std::cerr<< " Inside PeriodicSnapshotter constructor " << "\n";

    client_ = this->create_client<laser_assembler_srv_gen::srv::AssembleScans>("build_cloud");
    
    if (!client_->wait_for_service(20s)) {
      printf("Service not available after waiting");
      return;
    }
    printf(" Service assemble_scans started successfully");
    //ROS_INFO("Service [%s] detected", SERVICE_NAME.c_str());
    
    // Create a function for when messages are to be sent.
    auto timer_callback = [this]() -> void {
        // msg_->name = "Hello World: " + std::to_string(count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->status)
        //RCLCPP_INFO(this->get_logger(), "Publishing: namespce %s and name is %s ",
         // this->get_namespace(), this->get_name());

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        this->timerCallback();
      };
      
    // Create a publisher with a custom Quality of Service profile.
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
      "/assembled_cloud", custom_qos_profile);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(std::chrono::milliseconds(25), timer_callback);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  //void timerCallback(const builtin_interfaces::msg::Time& e)
  void timerCallback()
  {

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      last_time = rclcpp::Clock().now();
      first_time_ = false;
      return;
    }

    // Populate our service request based on our timer callback times
    auto request = std::make_shared<laser_assembler_srv_gen::srv::AssembleScans::Request>();
    
    request->begin = last_time.sec;
    last_time = rclcpp::Clock().now();
    request->end = last_time.sec;
      
      
    using ServiceResponseFuture =
        rclcpp::Client<laser_assembler_srv_gen::srv::AssembleScans>::SharedFuture;
      
      auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        printf("\n                                      Cloud points size = %ld\n", result.get()->cloud.points.size());
        //RCLCPP_INFO(this->get_logger(), "Got result: [%" PRId64 "]", future.get()->sum)
        if (result.get()->cloud.points.size() > 0 ) {
          
          result.get()->cloud;
          pub_->publish(result.get()->cloud);
        }
        else
          ROS_INFO("Got an empty cloud. Going to try again on the next scan");
      };
      
    auto result = client_->async_send_request(request);
}

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_;
  rclcpp::Client<laser_assembler_srv_gen::srv::AssembleScans>::SharedPtr client_;
  builtin_interfaces::msg::Time last_time;
  rclcpp::TimerBase::SharedPtr timer_;
  bool first_time_;
} ;

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto service_name = std::string("periodic_snapshotter");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
  if (nullptr != cli_option) {
    service_name = std::string(cli_option);
  }
  auto node = std::make_shared<PeriodicSnapshotter>(service_name);
  rclcpp::spin(node);
  return 0;
}
