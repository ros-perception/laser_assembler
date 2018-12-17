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


#include "laser_geometry/laser_geometry.hpp"

//#include "laser_assembler/laser_geometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_assembler/base_assembler.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <sstream>
#include <vector>
#include <chrono>
#include "filters/filter_chain.h"
#include "rclcpp/time.hpp"

using namespace laser_geometry;
using namespace std ;
using namespace std::chrono;

#define ROS_ERROR printf
#define ROS_INFO printf
#define ROS_WARN printf
#define ROS_DEBUG printf

#define TIME rclcpp::Time

namespace laser_assembler
{

/**
 * \brief Maintains a history of laser scans and generates a point cloud upon request
 */
class LaserScanAssembler : public BaseAssembler<sensor_msgs::msg::LaserScan>
{
public:
  LaserScanAssembler(rclcpp::Node::SharedPtr node_) : BaseAssembler<sensor_msgs::msg::LaserScan>("max_scans", node_), filter_chain_("sensor_msgs::msg::LaserScan")
  {
    // ***** Set Laser Projection Method *****
    private_ns_->get_parameter_or("ignore_laser_skew", ignore_laser_skew_, true);

    // configure the filter chain from the parameter server
    filter_chain_.configure("filters", private_ns_);

    // Have different callbacks, depending on whether or not we want to ignore laser skews.
    if (ignore_laser_skew_)
      start("scan");
    else
    {
      start();
      skew_scan_sub_ = n_->create_subscription<sensor_msgs::msg::LaserScan>("scan", std::bind(&LaserScanAssembler::scanCallback, this, std::placeholders::_1));
    }
  }

  ~LaserScanAssembler()
  {

  }

  unsigned int GetPointsInScan(const sensor_msgs::msg::LaserScan& scan)
  {
    return (scan.ranges.size ());
  }

 void ConvertToCloud(const string& fixed_frame_id, const sensor_msgs::msg::LaserScan& scan_in, sensor_msgs::msg::PointCloud& cloud_out)
  {
    // apply filters on laser scan
    filter_chain_.update (scan_in, scan_filtered_);

    int mask = laser_geometry::channel_option::Intensity +
                 laser_geometry::channel_option::Distance +
                 laser_geometry::channel_option::Index +
                 laser_geometry::channel_option::Timestamp;

    unsigned int n_pts = scan_in.ranges.size();
    
    // TODO ignore_laser_skew_ this case is not handled right now, as there is no transformPointCloud support for PointCloud in ROS2.
    if (ignore_laser_skew_)  // Do it the fast (approximate) way
    {
      projector_.projectLaser(scan_filtered_, cloud_out);
      if (cloud_out.header.frame_id != fixed_frame_id) {
          
        // TODO transform PointCloud
        /*
        tf_->transformPointCloud(fixed_frame_id, cloud_out, cloud_out);
        TIME start_time = cloud_out.header.stamp;
        std::chrono::nanoseconds start(start_time.nanoseconds());
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> st(start);
        geometry_msgs::msg::TransformStamped transform = tfBuffer.lookupTransform(fixed_frame_id,
        cloud_out.header.frame_id, st);
        sensor_msgs::msg::PointCloud2 cloudout;
        tf2::doTransform(cloud_out, cloud_out, transform);*/
      }	
    }
    else                     // Do it the slower (more accurate) way
    {
      int mask = laser_geometry::channel_option::Intensity +
                 laser_geometry::channel_option::Distance +
                 laser_geometry::channel_option::Index +
                 laser_geometry::channel_option::Timestamp;
      projector_.transformLaserScanToPointCloud (fixed_frame_id, scan_filtered_, cloud_out, tfBuffer, mask);
    }
    return;
  }

  void scanCallback(std::shared_ptr<sensor_msgs::msg::LaserScan const> laser_scan)
  {
    if (!ignore_laser_skew_)
    {
      rclcpp::Duration cur_tolerance = rclcpp::Duration(laser_scan->time_increment * laser_scan->ranges.size());
      if (cur_tolerance > max_tolerance_)
      {
        ROS_DEBUG("Upping tf tolerance from [%.4fs] to [%.4fs]", max_tolerance_.nanoseconds()/1e+9, cur_tolerance.nanoseconds()/1e+9);
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
  rclcpp::Duration max_tolerance_ = rclcpp::Duration(0, 0);   // The longest tolerance we've needed on a scan so far

  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;
  mutable sensor_msgs::msg::LaserScan scan_filtered_/*scan_filtered_*/;

};

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("laser_scan_assembler");
  LaserScanAssembler pc_assembler(node);
  rclcpp::spin(node);

  return 0;
}
