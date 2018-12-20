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
#include "laser_assembler/base_assembler.hpp"
// #include <sensor_msgs/msg/point_cloud_conversion.h> // TODO
#include "laser_assembler/point_cloud_conversion.hpp"

#define ROS_ERROR printf
#define ROS_INFO printf
#define ROS_WARN printf
#define ROS_DEBUG printf

namespace laser_assembler
{

/**
 * \brief Maintains a history of incremental point clouds (usually from laser
 * scans) and generates a point cloud upon request \todo Clean up the doxygen
 * part of this header params
 *  * (Several params are inherited from BaseAssemblerSrv)
 */
class PointCloud2Assembler
  : public BaseAssembler<sensor_msgs::msg::PointCloud2>
{
public:
  explicit PointCloud2Assembler(rclcpp::Node::SharedPtr node)
  : BaseAssembler<sensor_msgs::msg::PointCloud2>("max_clouds", node) {}

  ~PointCloud2Assembler() {}

  unsigned int GetPointsInScan(const sensor_msgs::msg::PointCloud2 & scan)
  {
    return scan.width * scan.height;
  }

  void ConvertToCloud(
    const string & fixed_frame_id,
    const sensor_msgs::msg::PointCloud2 & scan_in,
    sensor_msgs::msg::PointCloud & cloud_out)
  {
    sensor_msgs::msg::PointCloud cloud_in;
    sensor_msgs::convertPointCloud2ToPointCloud(scan_in, cloud_in);
    tf_->transformPointCloud(fixed_frame_id, cloud_in, cloud_out);
  }

private:
};

}  // namespace laser_assembler

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_cloud_assembler");
  laser_assembler::PointCloud2Assembler pc_assembler(node);
  pc_assembler.start("cloud");
  rclcpp::spin(node);

  return 0;
}
