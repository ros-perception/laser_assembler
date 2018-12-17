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


#include "laser_assembler/base_assembler.hpp"


using namespace std;

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
class PointCloudAssembler : public BaseAssembler<sensor_msgs::msg::PointCloud>
{
public:
  PointCloudAssembler(rclcpp::Node::SharedPtr node) : BaseAssembler<sensor_msgs::msg::PointCloud>("max_clouds", node)
  {

  }

  ~PointCloudAssembler()
  {

  }

  unsigned int GetPointsInScan(const sensor_msgs::msg::PointCloud& scan)
  {
    return (scan.points.size ());
  }

  void ConvertToCloud(const std::string& fixed_frame_id, const sensor_msgs::msg::PointCloud& scan_in, sensor_msgs::msg::PointCloud& cloud_out)
  {
    // TODO
    tf_->transformPointCloud(fixed_frame_id, scan_in, cloud_out) ;
    //tf2::doTransform(scan_in, cloud_out, tfBuffer.lookupTransform(fixed_frame_id, tf2::getFrameId(scan_in), tf2::getTimestamp(scan_in)));
    return ;
  }

private:

};

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_cloud_assembler");
  PointCloudAssembler pc_assembler(node);
  pc_assembler.start("cloud");
  rclcpp::spin(node);

  return 0;
}
