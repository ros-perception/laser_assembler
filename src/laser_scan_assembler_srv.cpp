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


#include <string>
#include "filters/filter_chain.h"
#include "geometry_msgs/msg/transform_stamped.hpp";
#include "laser_assembler/base_assembler_srv.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define ROS_ERROR printf
#define ROS_INFO printf
#define ROS_WARN printf
#define ROS_DEBUG printf

namespace laser_assembler
{
/**
 * \brief Maintains a history of laser scans and generates a point cloud upon
 * request \section params ROS Parameters
 * - (Several params are inherited from laser_assembler::BaseAssemblerSrv)
 * - \b "~ignore_laser_skew" (bool) - Specifies the method to project laser data
 *   - true -> Account for laser skew, and compute the transform for each laser
 * point (This is currently really slow!)
 *   - false-> Don't account for laser skew, and use 1 transform per scanline.
 * (This might have a little error when moving fast) \section services ROS
 * Services
 * - "~build_cloud" - Inhertited from laser_assembler::BaseAssemblerSrv
 */
class LaserScanAssemblerSrv
  : public BaseAssemblerSrv<sensor_msgs::msg::LaserScan>
{
public:
  LaserScanAssemblerSrv()
  : filter_chain_("sensor_msgs::msg::LaserScan")
  {
    // ***** Set Laser Projection Method *****
    private_ns_->get_parameter_or("ignore_laser_skew", ignore_laser_skew_,
      true);

    // configure the filter chain from the parameter server
    filter_chain_.configure("filters", private_ns_);
  }

  ~LaserScanAssemblerSrv() {}

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

    // convert laser scan to point cloud
    if (ignore_laser_skew_) {  // Do it the fast (approximate) way
      projector_.projectLaser(scan_filtered_, cloud_out);
      if (cloud_out.header.frame_id != fixed_frame_id) {
        tf_->transformPointCloud(fixed_frame_id, cloud_out, cloud_out);
      }
    } else {  // Do it the slower (more accurate) way
      int mask = laser_geometry::channel_option::Intensity +
        laser_geometry::channel_option::Distance +
        laser_geometry::channel_option::Index +
        laser_geometry::channel_option::Timestamp;
      projector_.transformLaserScanToPointCloud(fixed_frame_id, scan_filtered_,
        cloud_out, *tf_, mask);
    }
  }

private:
  bool ignore_laser_skew_;
  laser_geometry::LaserProjection projector_;

  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;
  mutable sensor_msgs::msg::LaserScan scan_filtered_;
};

}  // namespace laser_assembler

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "laser_scan_assembler");
  ros::NodeHandle n;
  ROS_WARN("The laser_scan_assembler_srv is deprecated. Please switch to "
    "using the laser_scan_assembler. Documentation is available at "
    "http://www.ros.org/wiki/laser_assembler");
  laser_assembler::LaserScanAssemblerSrv pc_assembler;
  pc_assembler.start();
  ros::spin();

  return 0;
}
