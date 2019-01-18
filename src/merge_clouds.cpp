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

/** \author Ioan Sucan */

/**

@b MergeClouds is a node capable of combining point clouds,
potentially from different sensors

 **/
#include <string>
#include <algorithm>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_assembler/message_filter.hpp"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

class MergeClouds
{
public:
  MergeClouds(void)
  : sub1_(nh_, "cloud_in1", 1), sub2_(nh_, "cloud_in1", 1)
  {
    cloudOut_ = nh_.advertise<sensor_msgs::msg::PointCloud>("cloud_out", 1);
    nh_.param<std::string>("~output_frame", output_frame_, std::string());
    nh_.param<double>("~max_frequency", max_freq_, 0.0);
    newCloud1_ = newCloud2_ = false;

    if (output_frame_.empty()) {
      ROS_ERROR("No output frame specified for merging pointclouds");
    }

    // make sure we don't publish too fast
    if (max_freq_ > 1000.0 || max_freq_ < 0.0) {
      max_freq_ = 0.0;
    }

    if (max_freq_ > 0.0) {
      timer_ = nh_.createTimer(rclcpp::Duration(1.0 / max_freq_),
          std::bind(&MergeClouds::onTimer, this, _1));
      haveTimer_ = true;
    } else {
      haveTimer_ = false;
    }

    tf_filter1_.reset(new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud>(
        sub1_, tf_, output_frame_, 1));
    tf_filter2_.reset(new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud>(
        sub2_, tf_, output_frame_, 1));

    tf_filter1_->registerCallback(
      std::bind(&MergeClouds::receiveCloud1, this, _1));
    tf_filter2_->registerCallback(
      std::bind(&MergeClouds::receiveCloud2, this, _1));
  }

  ~MergeClouds(void) {}

private:
  void onTimer(const ros::TimerEvent & e)
  {
    if (newCloud1_ && newCloud2_) {
      publishClouds();
    }
  }

  void publishClouds(void)
  {
    lock1_.lock();
    lock2_.lock();

    newCloud1_ = false;
    newCloud2_ = false;

    sensor_msgs::msg::PointCloud out;
    if (cloud1_.header.stamp > cloud2_.header.stamp) {
      out.header = cloud1_.header;
    } else {
      out.header = cloud2_.header;
    }

    out.points.resize(cloud1_.points.size() + cloud2_.points.size());

    // copy points
    std::copy(cloud1_.points.begin(), cloud1_.points.end(), out.points.begin());
    std::copy(cloud2_.points.begin(), cloud2_.points.end(),
      out.points.begin() + cloud1_.points.size());

    // copy common channels
    for (unsigned int i = 0; i < cloud1_.channels.size(); ++i) {
      for (unsigned int j = 0; j < cloud2_.channels.size(); ++j) {
        if (cloud1_.channels[i].name == cloud2_.channels[j].name) {
          ASSERT_TRUE(cloud1_.channels[i].values.size() == cloud1_.points.size());
          ASSERT_TRUE(cloud2_.channels[j].values.size() == cloud2_.points.size());
          unsigned int oc = out.channels.size();
          out.channels.resize(oc + 1);
          out.channels[oc].name = cloud1_.channels[i].name;
          out.channels[oc].values.resize(cloud1_.channels[i].values.size() +
            cloud2_.channels[j].values.size());
          std::copy(cloud1_.channels[i].values.begin(),
            cloud1_.channels[i].values.end(),
            out.channels[oc].values.begin());
          std::copy(cloud2_.channels[j].values.begin(),
            cloud2_.channels[j].values.end(),
            out.channels[oc].values.begin() +
            cloud1_.channels[i].values.size());
          break;
        }
      }
    }

    lock1_.unlock();
    lock2_.unlock();

    cloudOut_.publish(out);
  }

  void receiveCloud1(const sensor_msgs::msg::PointCloudConstPtr & cloud)
  {
    lock1_.lock();
    processCloud(cloud, cloud1_);
    lock1_.unlock();
    newCloud1_ = true;
    if (!haveTimer_ && newCloud2_) {
      publishClouds();
    }
  }

  void receiveCloud2(const sensor_msgs::msg::PointCloudConstPtr & cloud)
  {
    lock2_.lock();
    processCloud(cloud, cloud2_);
    lock2_.unlock();
    newCloud2_ = true;
    if (!haveTimer_ && newCloud1_) {
      publishClouds();
    }
  }

  void processCloud(
    const sensor_msgs::msg::PointCloudConstPtr & cloud,
    sensor_msgs::msg::PointCloud & cloudOut)
  {
    if (output_frame_ != cloud->header.frame_id) {
      tf_.transformPointCloud(output_frame_, *cloud, cloudOut);
    } else {
      cloudOut = *cloud;
    }
  }

  ros::NodeHandle nh_;
  tf::TransformListener tf_;

  ros::Timer timer_;
  bool haveTimer_;

  ros::Publisher cloudOut_;
  double max_freq_;
  std::string output_frame_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud> sub1_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud> sub2_;
  std::shared_ptr<tf::MessageFilter<sensor_msgs::msg::PointCloud>> tf_filter1_;
  std::shared_ptr<tf::MessageFilter<sensor_msgs::msg::PointCloud>> tf_filter2_;

  bool newCloud1_;
  bool newCloud2_;
  sensor_msgs::msg::PointCloud cloud1_;
  sensor_msgs::msg::PointCloud cloud2_;
  std::mutex lock1_;
  std::mutex lock2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("merge_clouds");
  MergeClouds mc;
  mc.start("cloud");
  rclcpp::spin(node);

  return 0;
}
