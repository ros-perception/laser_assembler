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
#include "laser_assembler/base_assembler_srv.hpp"

#define ROS_ERROR printf
#define ROS_INFO printf
#define ROS_WARN printf
#define ROS_DEBUG printf

namespace laser_assembler
{
/**
 * \brief Maintains a history of incremental point clouds (usually from laser scans) and generates a point cloud upon request
 * \todo Clean up the doxygen part of this header
 * params
 *  * (Several params are inherited from BaseAssemblerSrv)
 */
class PointCloudAssemblerSrv : public BaseAssemblerSrv<sensor_msgs::msg::PointCloud>
{
public:
  PointCloudAssemblerSrv()
  {
  }

  ~PointCloudAssemblerSrv()
  {
  }

  unsigned int GetPointsInScan(const sensor_msgs::msg::PointCloud & scan)
  {
    return scan.points.size();
  }

  void ConvertToCloud(
    const string & fixed_frame_id, const sensor_msgs::PointCloud & scan_in,
    sensor_msgs::PointCloud & cloud_out)
  {
    tf_->transformPointCloud(fixed_frame_id, scan_in, cloud_out);
  }

private:
};

}  // namespace laser_assembler

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "point_cloud_assembler");
  ros::NodeHandle n;
  ROS_WARN("The point_cloud_assembler_srv is deprecated. Please switch to "
    "using the laser_scan_assembler. Documentation is available at "
    "http://www.ros.org/wiki/laser_assembler");
  laser_assembler::PointCloudAssemblerSrv pc_assembler;
  pc_assembler.start();
  ros::spin();
  return 0;
}
