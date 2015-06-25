/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  point_cloud2_assembler.cpp
 *
 *  Created on: 01.07.2015
 *   Authors: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */


#include "laser_assembler/point_cloud2_base_assembler.h"
#include <tf/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


using namespace std ;

namespace laser_assembler
{

/**
 * \brief Maintains a history of incremental point clouds (usually from laser scans) and generates a point cloud upon request
 * \todo Clean up the doxygen part of this header
 * params
 *  * (Several params are inherited from BaseAssemblerSrv)
 */
class PointCloud2Assembler : public PointCloud2BaseAssembler<sensor_msgs::PointCloud2>
{
public:
  PointCloud2Assembler() : PointCloud2BaseAssembler<sensor_msgs::PointCloud2>("max_clouds")
  {
  }

  ~PointCloud2Assembler()
  {

  }

  unsigned int GetPointsInScan(const sensor_msgs::PointCloud2& scan)
  {
    return (scan.width * scan.height);
  }

  void Convert(const string& fixed_frame_id, const
    sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out)
  {

    std::string error_msg;
    bool success = this->tf_->waitForTransform(fixed_frame_id, cloud_in.header.frame_id, cloud_in.header.stamp,
        ros::Duration(0.1), ros::Duration(0.01), &error_msg);
    if (!success)
    {
      ROS_WARN("Could not get transform! %s", error_msg.c_str());
      return;
    }

    tf::StampedTransform transform;
    this->tf_->lookupTransform(fixed_frame_id, cloud_in.header.frame_id,  cloud_in.header.stamp, transform);
    geometry_msgs::TransformStamped tf_msg;
    tf::transformStampedTFToMsg(transform, tf_msg);
    tf2::doTransform(cloud_in, cloud_out, tf_msg);
  }

private:

};

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_assembler");
  PointCloud2Assembler pc_assembler;
  pc_assembler.start("cloud");
  ros::spin();

  return 0;
}
