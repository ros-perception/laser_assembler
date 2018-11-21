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
 *  point_cloud_base_assembler.h
 *
 *  Created on: 01.07.2015
 *   Authors: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef LASER_ASSEMBLER_POINT_CLOUD_BASE_ASSEMBLER_H
#define LASER_ASSEMBLER_POINT_CLOUD_BASE_ASSEMBLER_H

//! \author Sebastian Puetz

#include "base_assembler.h"
#include "sensor_msgs/PointCloud.h"

// Service
#include "laser_assembler/AssembleScans.h"

namespace laser_assembler
{

/**
 * \brief Maintains a history of point clouds and generates an aggregate point cloud upon request
 */
template<class T>
class PointCloudBaseAssembler : public BaseAssembler<T, sensor_msgs::PointCloud>
{
public:
  PointCloudBaseAssembler(const std::string& max_size_param_name);
  ~PointCloudBaseAssembler() ;

  /** \brief Returns the number of points in the current scan
   * \param scan The scan for for which we want to know the number of points
   * \return the number of points in scan
   */
  virtual unsigned int GetPointsInScan(const T& scan) = 0 ;

  /** \brief Converts the current scan into a cloud in the specified fixed frame
   *
   * Note: Once implemented, ConvertToCloud should NOT catch TF exceptions. These exceptions are caught by
   * BaseAssembler, and will be counted for diagnostic information
   * \param fixed_frame_id The name of the frame in which we want cloud_out to be in
   * \param scan_in The scan that we want to convert
   * \param cloud_out The result of transforming scan_in into a cloud in frame fixed_frame_id
   */
  virtual void Convert(const std::string& fixed_frame_id, const T& scan_in, sensor_msgs::PointCloud& cloud_out) = 0 ;

private:
  // ROS Input/Ouptut Handling
  ros::ServiceServer build_cloud_server_;
  ros::ServiceServer assemble_scans_server_;

  //! \brief Service Callback function called whenever we need to build a cloud
  bool buildCloud(AssembleScans::Request& req, AssembleScans::Response& resp) ;
  bool assembleScans(AssembleScans::Request& req, AssembleScans::Response& resp) ;
} ;

template <class T>
PointCloudBaseAssembler<T>::PointCloudBaseAssembler(const std::string& max_size_param_name) 
  : BaseAssembler<T, sensor_msgs::PointCloud>(max_size_param_name)
{
  // ***** Start Services *****
  build_cloud_server_    = PointCloudBaseAssembler<T>::n_.advertiseService("build_cloud", &PointCloudBaseAssembler<T>::buildCloud,    this);
  assemble_scans_server_ = PointCloudBaseAssembler<T>::n_.advertiseService("assemble_scans", &PointCloudBaseAssembler<T>::assembleScans, this);
}

template <class T>
PointCloudBaseAssembler<T>::~PointCloudBaseAssembler()
{

}

template <class T>
bool PointCloudBaseAssembler<T>::buildCloud(AssembleScans::Request& req, AssembleScans::Response& resp)
{
  ROS_WARN("Service 'build_cloud' is deprecated. Call 'assemble_scans' instead");
  return assembleScans(req, resp);
}


template <class T>
bool PointCloudBaseAssembler<T>::assembleScans(AssembleScans::Request& req, AssembleScans::Response& resp)
{
  this->scan_hist_mutex_.lock() ;
  // Determine where in our history we actually are
  unsigned int i = 0 ;

  // Find the beginning of the request. Probably should be a search
  while ( i < BaseAssembler<T, sensor_msgs::PointCloud>::scan_hist_.size() &&                                                    // Don't go past end of deque
          BaseAssembler<T, sensor_msgs::PointCloud>::scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
  {
    i++ ;
  }
  unsigned int start_index = i ;

  unsigned int req_pts = 0 ;                                                          // Keep a total of the points in the current request
  // Find the end of the request
  while ( i < BaseAssembler<T,sensor_msgs::PointCloud>::scan_hist_.size() &&                                                    // Don't go past end of deque
          BaseAssembler<T, sensor_msgs::PointCloud>::scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
  {
    req_pts += (BaseAssembler<T,
    sensor_msgs::PointCloud>::scan_hist_[i].points.size ()+BaseAssembler<T,
    sensor_msgs::PointCloud>::downsample_factor_-1)/ BaseAssembler<T, sensor_msgs::PointCloud>::downsample_factor_ ;
    i += BaseAssembler<T, sensor_msgs::PointCloud>::downsample_factor_ ;
  }
  unsigned int past_end_index = i ;

  if (start_index == past_end_index)
  {
    resp.cloud.header.frame_id = this->fixed_frame_ ;
    resp.cloud.header.stamp = req.end ;
    resp.cloud.points.resize (0) ;
    resp.cloud.channels.resize (0) ;
  }
  else
  {
    // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
    // Allocate space for the cloud
    resp.cloud.points.resize (req_pts);
    const unsigned int num_channels = this->scan_hist_[start_index].channels.size ();
    resp.cloud.channels.resize (num_channels) ;
    for (i = 0; i<num_channels; i++)
    {
      resp.cloud.channels[i].name = this->scan_hist_[start_index].channels[i].name ;
      resp.cloud.channels[i].values.resize (req_pts) ;
    }
    //resp.cloud.header.stamp = req.end ;
    resp.cloud.header.frame_id = this->fixed_frame_ ;
    unsigned int cloud_count = 0 ;
    for (i=start_index; i<past_end_index; i+= BaseAssembler<T, sensor_msgs::PointCloud>::downsample_factor_)
    {

      // Sanity check: Each channel should be the same length as the points vector
      for (unsigned int chan_ind = 0; chan_ind < BaseAssembler<T, sensor_msgs::PointCloud>::scan_hist_[i].channels.size(); chan_ind++)
      {
        if (this->scan_hist_[i].points.size () != this->scan_hist_[i].channels[chan_ind].values.size())
          ROS_FATAL("Trying to add a malformed point cloud. Cloud has %u points, but channel %u has %u elems",
          (int)this->scan_hist_[i].points.size (), chan_ind, (int)this->scan_hist_[i].channels[chan_ind].values.size ());
      }

      for(unsigned int j=0; j<this->scan_hist_[i].points.size (); j+=this->downsample_factor_)
      {
        resp.cloud.points[cloud_count].x = this->scan_hist_[i].points[j].x ;
        resp.cloud.points[cloud_count].y = this->scan_hist_[i].points[j].y ;
        resp.cloud.points[cloud_count].z = this->scan_hist_[i].points[j].z ;

        for (unsigned int k=0; k<num_channels; k++)
          resp.cloud.channels[k].values[cloud_count] = this->scan_hist_[i].channels[k].values[j] ;

        cloud_count++ ;
      }
      resp.cloud.header.stamp = this->scan_hist_[i].header.stamp;
    }
  }
  this->scan_hist_mutex_.unlock() ;

  ROS_DEBUG("Point Cloud Results: Aggregated from index %u->%u. BufferSize: %lu. Points in cloud: %u", start_index, past_end_index, this->scan_hist_.size(), (int)resp.cloud.points.size ()) ;
  return true ;
}

}

#endif /* point_cloud_base_assembler */
