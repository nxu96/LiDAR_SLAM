/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 17:50:26
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 18:07:47
 * @Description: Keyframes path subscriber
 */
#include "subscriber/key_frames_subscriber.h"
#include <glog/logging.h>

namespace lidar_slam {
KeyFramesSubscriber::KeyFramesSubscriber(ros::NodeHandle& nh,
                                         std::string topic_name,
                                         size_t buff_size) :
    nh_(nh) {
  sub_ = nh_.subscribe(topic_name, buff_size,
                       &KeyFramesSubscriber::msg_callback, this);
}

void KeyFramesSubscriber::ParseData(std::deque<KeyFrame>& deque_key_frames) {
  if (!new_key_frame_.empty()) {
    // TODO(nxu): Why not inserting but replacing here?
    deque_key_frames = new_key_frame_;
    new_key_frame_.clear();
  }
}

void KeyFramesSubscriber::msg_callback(
    const nav_msgs::Path::ConstPtr& key_frames_msg_ptr) {
  new_key_frame_.clear();
  for (size_t i = 0; i < key_frames_msg_ptr->poses.size(); ++i) {
    KeyFrame kf;
    kf.time = key_frames_msg_ptr->poses.at(i).header.stamp.toSec();
    kf.index = key_frames_msg_ptr->poses.at(i).header.seq;

    kf.pose(0, 3) = key_frames_msg_ptr->poses.at(i).pose.position.x;
    kf.pose(1, 3) = key_frames_msg_ptr->poses.at(i).pose.position.y;
    kf.pose(2, 3) = key_frames_msg_ptr->poses.at(i).pose.position.z;

    Eigen::Quaternionf q;
    q.x() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
    q.y() = key_frames_msg_ptr->poses.at(i).pose.orientation.y;
    q.z() = key_frames_msg_ptr->poses.at(i).pose.orientation.z;
    q.w() = key_frames_msg_ptr->poses.at(i).pose.orientation.w;

    kf.pose.block<3, 3>(0, 0) = q.matrix();

    new_key_frame_.push_back(kf);
  }
}

}  // namespace lidar_slam
