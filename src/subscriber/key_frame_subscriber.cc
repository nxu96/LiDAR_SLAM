/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 17:33:14
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 17:46:51
 * @Description: Single Key frame subscriber implementation
 */
#include "subscriber/key_frame_subscriber.h"
#include <glog/logging.h>

namespace lidar_slam {
KeyFrameSubscriber::KeyFrameSubscriber(ros::NodeHandle& nh,
                                       std::string topic_name, size_t buff_size)
    : nh_(nh) {
  sub_ = nh_.subscribe(topic_name, buff_size, &KeyFrameSubscriber::msg_callback,
                       this);
}

void KeyFrameSubscriber::ParseData(std::deque<KeyFrame>& key_frame_buff) {
  if (!new_key_frame_.empty()) {
    key_frame_buff.insert(key_frame_buff.end(),
                          new_key_frame_.begin(), new_key_frame_.end());
    new_key_frame_.clear();
  }
}

void KeyFrameSubscriber::msg_callback(
    const geometry_msgs::PoseStampedConstPtr& key_frame_msg_ptr) {
  KeyFrame key_frame;
  key_frame.time = key_frame_msg_ptr->header.stamp.toSec();
  key_frame.index = key_frame_msg_ptr->header.seq;

  key_frame.pose(0, 3) = key_frame_msg_ptr->pose.position.x;
  key_frame.pose(1, 3) = key_frame_msg_ptr->pose.position.y;
  key_frame.pose(2, 3) = key_frame_msg_ptr->pose.position.z;

  Eigen::Quaternionf q;
  q.x() = key_frame_msg_ptr->pose.orientation.x;
  q.y() = key_frame_msg_ptr->pose.orientation.y;
  q.z() = key_frame_msg_ptr->pose.orientation.z;
  q.w() = key_frame_msg_ptr->pose.orientation.w;

  key_frame.pose.block<3, 3>(0, 0) = q.matrix();

  new_key_frame_.push_back(key_frame);
}
}  // namespace lidar_slam
