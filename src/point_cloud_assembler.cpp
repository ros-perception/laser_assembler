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

namespace laser_assembler
{

/**
 * \brief Maintains a history of incremental point clouds (usually from laser
 * scans) and generates a point cloud upon request \todo Clean up the doxygen
 * part of this header params
 *  * (Several params are inherited from BaseAssemblerSrv)
 */
class PointCloudAssembler : public BaseAssembler<sensor_msgs::msg::PointCloud>
{
public:
  explicit PointCloudAssembler(rclcpp::Node::SharedPtr node)
  : BaseAssembler<sensor_msgs::msg::PointCloud>("max_clouds", node) {}

  ~PointCloudAssembler() {}

  unsigned int GetPointsInScan(const sensor_msgs::msg::PointCloud & scan)
  {
    return scan.points.size();
  }

  void ConvertToCloud(
    const std::string & fixed_frame_id,
    const sensor_msgs::msg::PointCloud & scan_in,
    sensor_msgs::msg::PointCloud & cloud_out)
  {
    tf_->transformPointCloud(fixed_frame_id, scan_in, cloud_out);
    // tf2::doTransform(scan_in, cloud_out,
    // tfBuffer.lookupTransform(fixed_frame_id, tf2::getFrameId(scan_in),
    // tf2::getTimestamp(scan_in)));
  }

private:
};

}  // namespace laser_assembler

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_cloud_assembler");
  laser_assembler::PointCloudAssembler pc_assembler(node);
  pc_assembler.start("cloud");
  rclcpp::spin(node);

  return 0;
}
