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


#include <chrono>
#include <cstdio>
#include <string>
#include <memory>
// Services
#include "laser_assembler_srv_gen/srv/assemble_scans.hpp"

#include "rclcpp/rclcpp.hpp"
// Messages
#include "rclcpp/clock.hpp"
#include "rcutils/cmdline_parser.h"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */
namespace laser_assembler
{

class PeriodicSnapshotter : public rclcpp::Node
{
public:
  explicit PeriodicSnapshotter(const std::string & service_name)
  : Node(service_name)
  {
    static const char SERVICE_NAME[] = "assemble_scans";
    client_ = this->create_client<laser_assembler_srv_gen::srv::AssembleScans>(
      SERVICE_NAME);

    using namespace std::chrono_literals;
    if (!client_->wait_for_service(20s)) {
      RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service [%s] detected", SERVICE_NAME);

    // Create a function for when messages are to be sent.
    auto timer_callback = [this]() -> void {
        this->timerCallback();
      };

    // Create a publisher with a custom Quality of Service profile.
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
      "/assembled_cloud", custom_qos_profile);

    // Use a timer to schedule periodic message publishing.
    timer_ =
      this->create_wall_timer(std::chrono::milliseconds(25), timer_callback);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  // void timerCallback(const builtin_interfaces::msg::Time& e)
  void timerCallback()
  {
    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_) {
      last_time = rclcpp::Clock().now();
      first_time_ = false;
      return;
    }

    // Populate our service request based on our timer callback times
    auto request = std::make_shared<
      laser_assembler_srv_gen::srv::AssembleScans::Request>();

    request->begin = last_time.sec;
    last_time = rclcpp::Clock().now();
    request->end = last_time.sec;

    using ServiceResponseFuture = rclcpp::Client<
      laser_assembler_srv_gen::srv::AssembleScans>::SharedFuture;

    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        if (result.get()->cloud.points.size() > 0) {
          RCLCPP_INFO(this->get_logger(), "Got result: [ %ld ]", result.get()->cloud.points.size());
          pub_->publish(result.get()->cloud);
        } else {
          RCLCPP_INFO(this->get_logger(), "Got an empty cloud. Going to try again on the next scan");
        }
      };

    auto result = client_->async_send_request(request);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_;
  rclcpp::Client<laser_assembler_srv_gen::srv::AssembleScans>::SharedPtr
    client_;
  builtin_interfaces::msg::Time last_time;
  rclcpp::TimerBase::SharedPtr timer_;
  bool first_time_;
};

}  // namespace laser_assembler

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto service_name = std::string("periodic_snapshotter");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
  if (nullptr != cli_option) {
    service_name = std::string(cli_option);
  }
  auto node = std::make_shared<laser_assembler::PeriodicSnapshotter>(service_name);
  rclcpp::spin(node);
  return 0;
}
