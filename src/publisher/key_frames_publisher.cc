/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 16:51:37
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 17:03:33
 * @Description: Description
 */
#include "publisher/key_frames_publisher.h"
#include <Eigen/Dense>

namespace lidar_slam {
KeyFramesPublisher::KeyFramesPublisher(ros::NodeHandle& nh,
                                       std::string topic_name,
                                       std::string frame_id, size_t buff_size) :
    nh_(nh), frame_id_(frame_id) {
  pub_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
}

void KeyFramesPublisher::Publish(const std::deque<KeyFrame>& key_frames) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = frame_id_;
  KeyFrame curr_kf;
  geometry_msgs::PoseStamped curr_pose;
  for (size_t i = 0; i < key_frames.size(); ++i) {
    curr_kf = key_frames.at(i);
    ros::Time ros_time(static_cast<float>(curr_kf.time));

    curr_pose.header.stamp = ros_time;
    curr_pose.header.frame_id = frame_id_;
    curr_pose.header.frame_id = curr_kf.index;

    curr_pose.pose.position.x = curr_kf.pose(0, 3);
    curr_pose.pose.position.y = curr_kf.pose(1, 3);
    curr_pose.pose.position.z = curr_kf.pose(2, 3);

    Eigen::Quaternionf q = curr_kf.GetQuaternion();

    curr_pose.pose.orientation.w = q.w();
    curr_pose.pose.orientation.x = q.x();
    curr_pose.pose.orientation.y = q.y();
    curr_pose.pose.orientation.z = q.z();

    path.poses.push_back(curr_pose);
  }

  pub_.publish(path);
}

bool KeyFramesPublisher::HasSubscribers() {
  return pub_.getNumSubscribers() != 0;
}
}  // namespace lidar_slam
