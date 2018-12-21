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

#include <memory>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include "laser_geometry/laser_geometry.hpp"

// #include "laser_assembler/laser_geometry.hpp"
#include "filters/filter_chain.h"
#include "laser_assembler/base_assembler.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#define TIME rclcpp::Time

namespace laser_assembler
{

/**
 * \brief Maintains a history of laser scans and generates a point cloud upon
 * request
 */
class LaserScanAssembler : public BaseAssembler<sensor_msgs::msg::LaserScan>
{
public:
  explicit LaserScanAssembler(rclcpp::Node::SharedPtr node_)
  : BaseAssembler<sensor_msgs::msg::LaserScan>("max_scans", node_),
    filter_chain_("sensor_msgs::msg::LaserScan")
  {
    // ***** Set Laser Projection Method *****
    n_->get_parameter_or("ignore_laser_skew", ignore_laser_skew_,
      true);

    // configure the filter chain from the parameter server
    filter_chain_.configure("filters", n_);

    // Have different callbacks, depending on whether or not we want to ignore
    // laser skews.
    if (ignore_laser_skew_) {
      start("scan");
    } else {
      start();
      skew_scan_sub_ = n_->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", std::bind(&LaserScanAssembler::scanCallback, this,
        std::placeholders::_1));
    }
  }

  ~LaserScanAssembler() {}

  unsigned int GetPointsInScan(const sensor_msgs::msg::LaserScan & scan)
  {
    return scan.ranges.size();
  }

  void ConvertToCloud(
    const string & fixed_frame_id,
    const sensor_msgs::msg::LaserScan & scan_in,
    sensor_msgs::msg::PointCloud & cloud_out)
  {
    // apply filters on laser scan
    filter_chain_.update(scan_in, scan_filtered_);

    int mask = laser_geometry::channel_option::Intensity +
      laser_geometry::channel_option::Distance +
      laser_geometry::channel_option::Index +
      laser_geometry::channel_option::Timestamp;

    unsigned int n_pts = scan_in.ranges.size();

    // TODO(vandana) ignore_laser_skew_ this case is not handled right now, as there is
    // no transformPointCloud support for PointCloud in ROS2.
    if (ignore_laser_skew_) {  // Do it the fast (approximate) way
      projector_.projectLaser(scan_filtered_, cloud_out);
      if (cloud_out.header.frame_id != fixed_frame_id) {
        // TODO(vandana) transform PointCloud
        /*
        tf_->transformPointCloud(fixed_frame_id, cloud_out, cloud_out);
        TIME start_time = cloud_out.header.stamp;
        std::chrono::nanoseconds start(start_time.nanoseconds());
        std::chrono::time_point<std::chrono::system_clock,
        std::chrono::nanoseconds> st(start);
        geometry_msgs::msg::TransformStamped transform =
        tfBuffer.lookupTransform(fixed_frame_id, cloud_out.header.frame_id, st);
        sensor_msgs::msg::PointCloud2 cloudout;
        tf2::doTransform(cloud_out, cloud_out, transform);*/
      }
    } else {  // Do it the slower (more accurate) way
      int mask = laser_geometry::channel_option::Intensity +
        laser_geometry::channel_option::Distance +
        laser_geometry::channel_option::Index +
        laser_geometry::channel_option::Timestamp;
      projector_.transformLaserScanToPointCloud(fixed_frame_id, scan_filtered_,
        cloud_out, tfBuffer, mask);
    }
  }

  void
  scanCallback(std::shared_ptr<sensor_msgs::msg::LaserScan const> laser_scan)
  {
    if (!ignore_laser_skew_) {
      rclcpp::Duration cur_tolerance = rclcpp::Duration(
        laser_scan->time_increment * laser_scan->ranges.size());
      if (cur_tolerance > max_tolerance_) {
        RCLCPP_DEBUG(n_->get_logger(), "Upping tf tolerance from [%.4fs] to [%.4fs]",
          max_tolerance_.nanoseconds() / 1e+9,
          cur_tolerance.nanoseconds() / 1e+9);
        assert(tf_filter_);
        tf_filter_->setTolerance(cur_tolerance);
        max_tolerance_ = cur_tolerance;
      }
      tf_filter_->add(laser_scan);
    }
  }

private:
  bool ignore_laser_skew_;
  laser_geometry::LaserProjection projector_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr skew_scan_sub_;
  rclcpp::Duration max_tolerance_ = rclcpp::Duration(
    0, 0);   // The longest tolerance we've needed on a scan so far

  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;
  mutable sensor_msgs::msg::LaserScan scan_filtered_ /*scan_filtered_*/;
};

}  // namespace laser_assembler

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("laser_scan_assembler");
  laser_assembler::LaserScanAssembler pc_assembler(node);
  rclcpp::spin(node);

  return 0;
}
