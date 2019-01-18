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

//! \author Vijay Pradeep

#ifndef LASER_ASSEMBLER__BASE_ASSEMBLER_HPP_
#define LASER_ASSEMBLER__BASE_ASSEMBLER_HPP_

#include <cmath>
#include <string>
#include <memory>
#include <mutex>
#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
// #include "tf2_ros/message_filter.h" // TODO message_filter.h file in bouncy's
// tf2_ros package is not ported to ros2 yet. Found ported message_filter.h file
// on link
// https://github.com/ros2/geometry2/blob/ros2/tf2_ros/include/tf2_ros/message_filter.h
// so copied that file to include directory of this package and fixed some
// errros.
#include "laser_assembler/message_filter.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
// #include "sensor_msgs/point_cloud_conversion.h" // TODO This file was not
// there in sensor_msgs package of ros2 so just copied it from ROS1 and ported to
// ROS2 here.
#include "message_filters/subscriber.h"
#include "laser_assembler/point_cloud_conversion.hpp"

// Service
#include "laser_assembler/srv/assemble_scans.hpp"
#include "laser_assembler/srv/assemble_scans2.hpp"

rclcpp::Logger g_logger = rclcpp::get_logger("laser_scan_assembler");

namespace laser_assembler
{

/**
 * \brief Maintains a history of point clouds and generates an aggregate point
 * cloud upon request
 */
template<class T>
class BaseAssembler
{
public:
  BaseAssembler(
    const std::string & max_size_param_name,
    const rclcpp::Node::SharedPtr & node_);
  ~BaseAssembler();

  /**
   * \brief Tells the assembler to start listening to input data
   * It is possible that a derived class needs to initialize and configure
   * various things before actually beginning to process scans. Calling start
   * create subcribes to the input stream, thus allowing the scanCallback and
   * ConvertToCloud to be called. Start should be called only once.
   */
  void start(const std::string & in_topic_name);
  void start();

  /** \brief Returns the number of points in the current scan
   * \param scan The scan for for which we want to know the number of points
   * \return the number of points in scan
   */
  virtual unsigned int GetPointsInScan(const T & scan) = 0;

  /** \brief Converts the current scan into a cloud in the specified fixed frame
   *
   * Note: Once implemented, ConvertToCloud should NOT catch TF exceptions.
   * These exceptions are caught by BaseAssembler, and will be counted for
   * diagnostic information \param fixed_frame_id The name of the frame in which
   * we want cloud_out to be in \param scan_in The scan that we want to convert
   * \param cloud_out The result of transforming scan_in into a cloud in frame
   * fixed_frame_id
   */
  virtual void ConvertToCloud(
    const std::string & fixed_frame_id,
    const T & scan_in,
    sensor_msgs::msg::PointCloud & cloud_out) = 0;

protected:
  // message_filters's subscribe method requires raw node pointer.
  rclcpp::Node::SharedPtr n_;
  tf2::BufferCore tfBuffer;
  tf2_ros::TransformListener * tf_;
  tf2_ros::MessageFilter<T> * tf_filter_;

private:
  // ROS Input/Ouptut Handling
  rclcpp::Service<laser_assembler::srv::AssembleScans>::SharedPtr
    assemble_scans_server_;

  message_filters::Subscriber<T> scan_sub_;

  //! \brief Callback function for every time we receive a new scan
  virtual void msgCallback(const std::shared_ptr<const T> & scan_ptr);

  //! \brief Service Callback function called whenever we need to build a cloud
  bool assembleScans(
    std::shared_ptr<laser_assembler::srv::AssembleScans::Request>
    request,
    std::shared_ptr<laser_assembler::srv::AssembleScans::Response>
    response);

  //! \brief Stores history of scans
  std::deque<sensor_msgs::msg::PointCloud> scan_hist_;
  std::mutex scan_hist_mutex_;

  //! \brief The number points currently in the scan history
  unsigned int total_pts_;

  //! \brief The max number of scans to store in the scan history
  unsigned int max_scans_;

  //! \brief The frame to transform data into upon receipt
  std::string fixed_frame_;

  //! \brief Specify how much to downsample the data. A value of 1 preserves all
  //! the data. 3 would keep 1/3 of the data.
  unsigned int downsample_factor_;
};

template<class T>
BaseAssembler<T>::BaseAssembler(
  const std::string & max_size_param_name,
  const rclcpp::Node::SharedPtr & node_)
{
  // **** Initialize TransformListener ****
  double tf_cache_time_secs;
  n_ = node_;

  n_->get_parameter_or("tf_cache_time_secs", tf_cache_time_secs, 10.0);

  if (tf_cache_time_secs < 0) {
    RCLCPP_ERROR(g_logger, "Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs);
  }

  tf_ = new tf2_ros::TransformListener(tfBuffer);
  RCLCPP_INFO(g_logger, "TF Cache Time: %f Seconds ", tf_cache_time_secs);

  // ***** Set max_scans *****
  const int default_max_scans = 400;
  int tmp_max_scans;

  n_->get_parameter_or(max_size_param_name, tmp_max_scans,
    default_max_scans);

  if (tmp_max_scans < 0) {
    RCLCPP_ERROR(g_logger, "Parameter max_scans<0 (%i)", tmp_max_scans);
    tmp_max_scans = default_max_scans;
  }
  max_scans_ = tmp_max_scans;
  RCLCPP_INFO(g_logger, "Max Scans in History: %u ", max_scans_);
  total_pts_ = 0;  // We're always going to start with no points in our history

  // ***** Set fixed_frame *****
  n_->get_parameter_or("fixed_frame", fixed_frame_,
    std::string("ERROR_NO_NAME"));

  RCLCPP_INFO(g_logger, "Fixed Frame: %s ", fixed_frame_.c_str());

  if (fixed_frame_ == "ERROR_NO_NAME") {
    RCLCPP_ERROR(g_logger, "Need to set parameter fixed_frame");
  }

  // ***** Set downsample_factor *****
  int tmp_downsample_factor;
  n_->get_parameter_or("downsample_factor", tmp_downsample_factor, 1);
  if (tmp_downsample_factor < 1) {
    RCLCPP_ERROR(g_logger, "Parameter downsample_factor<1: %i", tmp_downsample_factor);
    tmp_downsample_factor = 1;
  }
  downsample_factor_ = tmp_downsample_factor;
  if (downsample_factor_ != 1) {
    RCLCPP_WARN(g_logger, "Downsample set to [%u]. Note that this is an unreleased/unstable "
      "feature",
      downsample_factor_);
  }

  // ***** Start Services *****
  auto assembleScansCallback =
    [this](
    std::shared_ptr<laser_assembler::srv::AssembleScans::Request>
    request,
    std::shared_ptr<laser_assembler::srv::AssembleScans::Response>
    response) -> bool {
      this->assembleScans(request, response);
      return true;
    };

  assemble_scans_server_ =
    n_->create_service<laser_assembler::srv::AssembleScans>(
    "assemble_scans", assembleScansCallback);

  // ***** Start Listening to Data *****
  // (Well, don't start listening just yet. Keep this as null until we actually
  // start listening, when start() is called)
  tf_filter_ = NULL;
}

template<class T>
void BaseAssembler<T>::start(const std::string & in_topic_name)
{
  RCLCPP_DEBUG(g_logger, "Called start(string). Starting to listen on "
    "message_filter::Subscriber the input stream");
  if (tf_filter_) {
    RCLCPP_ERROR(g_logger, "assembler::start() was called twice!. This is bad, and could "
      "leak memory");
  } else {
    // subscribe method requires raw node pointer.
    scan_sub_.subscribe(n_.get(), in_topic_name);
    tf_filter_ =
      new tf2_ros::MessageFilter<T>(scan_sub_, tfBuffer, fixed_frame_, 10, 0);
    tf_filter_->registerCallback(
      std::bind(&BaseAssembler<T>::msgCallback, this, std::placeholders::_1));
  }
}

template<class T>
void BaseAssembler<T>::start()
{
  RCLCPP_DEBUG(g_logger, "Called start(). Starting tf::MessageFilter, but not initializing "
    "Subscriber");
  if (tf_filter_) {
    RCLCPP_ERROR(g_logger, "assembler::start() was called twice!. This is bad, and could "
      "leak memory");
  } else {
    scan_sub_.subscribe(n_.get(), "bogus");
    tf_filter_ =
      new tf2_ros::MessageFilter<T>(scan_sub_, tfBuffer, fixed_frame_, 10, 0);
    tf_filter_->registerCallback(
      std::bind(&BaseAssembler<T>::msgCallback, this, std::placeholders::_1));
  }
}

template<class T>
BaseAssembler<T>::~BaseAssembler()
{
  if (tf_filter_) {
    delete tf_filter_;
  }

  delete tf_;
}

template<class T>
void BaseAssembler<T>::msgCallback(const std::shared_ptr<const T> & scan_ptr)
{
  RCLCPP_DEBUG(g_logger, "starting msgCallback");
  const T scan = *scan_ptr;

  sensor_msgs::msg::PointCloud cur_cloud;

  // Convert the scan data into a universally known datatype: PointCloud
  try {
    ConvertToCloud(fixed_frame_, scan,
      cur_cloud);              // Convert scan into a point cloud
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(g_logger, "Transform Exception %s", ex.what());
    return;
  }

  // Add the current scan (now of type PointCloud) into our history of scans
  scan_hist_mutex_.lock();
  if (scan_hist_.size() == max_scans_) {  // Is our deque full?
    total_pts_ -=
      scan_hist_.front().points.size();   // We're removing an elem, so this
                                          // reduces our total point count
    scan_hist_.pop_front();  // The front of the deque has the oldest elem, so we
                             // can get rid of it
  }
  scan_hist_.push_back(
    cur_cloud);   // Add the newest scan to the back of the deque
  total_pts_ += cur_cloud.points
    .size();                 // Add the new scan to the running total of points

  // printf("Scans: %4lu  Points: %10u\n", scan_hist_.size(), total_pts_);

  scan_hist_mutex_.unlock();
  RCLCPP_DEBUG(g_logger, "done with msgCallback");
}

template<class T>
bool BaseAssembler<T>::assembleScans(
  std::shared_ptr<laser_assembler::srv::AssembleScans::Request> req,
  std::shared_ptr<laser_assembler::srv::AssembleScans::Response>
  resp)
{
  scan_hist_mutex_.lock();
  // Determine where in our history we actually are
  unsigned int i = 0;

  if (scan_hist_.size() <= 0) {
    return true;
  }

  // Find the beginning of the request. Probably should be a search
  while (i < scan_hist_.size() &&  // Don't go past end of deque
    scan_hist_[i].header.stamp.sec <
    req->begin)          // Keep stepping until we've exceeded the start time
  {
    i++;
  }
  unsigned int start_index = i;

  unsigned int req_pts = 0;  // Keep a total of the points in the current request
  // Find the end of the request
  while (i < scan_hist_.size() &&  // Don't go past end of deque
    ((scan_hist_[i].header.stamp.sec) <
    req->end))       // Don't go past the end-time of the request
  {
    req_pts += (scan_hist_[i].points.size() + downsample_factor_ - 1) /
      downsample_factor_;
    i += downsample_factor_;
  }
  unsigned int past_end_index = i;

  if (start_index == past_end_index) {
    resp->cloud.header.frame_id = fixed_frame_;
    resp->cloud.header.stamp.sec = req->end;
    resp->cloud.points.resize(0);
    resp->cloud.channels.resize(0);
  } else {
    // Note: We are assuming that channel information is consistent across
    // multiple scans. If not, then bad things (segfaulting) will happen
    // Allocate space for the cloud
    resp->cloud.points.resize(req_pts);
    const unsigned int num_channels = scan_hist_[start_index].channels.size();
    resp->cloud.channels.resize(num_channels);
    for (i = 0; i < num_channels; i++) {
      resp->cloud.channels[i].name = scan_hist_[start_index].channels[i].name;
      resp->cloud.channels[i].values.resize(req_pts);
    }
    // resp.cloud.header.stamp = req.end ;
    resp->cloud.header.frame_id = fixed_frame_;
    unsigned int cloud_count = 0;
    for (i = start_index; i < past_end_index; i += downsample_factor_) {
      // Sanity check: Each channel should be the same length as the points
      // vector
      for (unsigned int chan_ind = 0; chan_ind < scan_hist_[i].channels.size();
        chan_ind++)
      {
        if (scan_hist_[i].points.size() !=
          scan_hist_[i].channels[chan_ind].values.size())
        {
          RCLCPP_FATAL(g_logger, "Trying to add a malformed point cloud. Cloud has %u "
            "points, but channel %u has %u elems",
            (int)scan_hist_[i].points.size(), chan_ind,
            (int)scan_hist_[i].channels[chan_ind].values.size());
        }
      }

      for (unsigned int j = 0; j < scan_hist_[i].points.size();
        j += downsample_factor_)
      {
        resp->cloud.points[cloud_count].x = scan_hist_[i].points[j].x;
        resp->cloud.points[cloud_count].y = scan_hist_[i].points[j].y;
        resp->cloud.points[cloud_count].z = scan_hist_[i].points[j].z;

        for (unsigned int k = 0; k < num_channels; k++) {
          resp->cloud.channels[k].values[cloud_count] =
            scan_hist_[i].channels[k].values[j];
        }

        cloud_count++;
      }
      resp->cloud.header.stamp = scan_hist_[i].header.stamp;
    }
  }
  scan_hist_mutex_.unlock();

  RCLCPP_DEBUG(g_logger, "\nPoint Cloud Results: Aggregated from index %u->%u. BufferSize: "
    "%lu. Points in cloud: %u",
    start_index, past_end_index, scan_hist_.size(),
    (int)resp->cloud.points.size());
  return true;
}
}  // namespace laser_assembler
#endif  // LASER_ASSEMBLER__BASE_ASSEMBLER_HPP_
