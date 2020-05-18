/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 18:08:26
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 18:23:08
 * @Description: Odometry subscriber implementation
 */
#include "subscriber/odometry_subscriber.h"

namespace lidar_slam {
OdometrySubscriber::OdometrySubscriber(ros::NodeHandle& nh,
                                       std::string topic_name,
                                       size_t buff_size) : nh_(nh) {
  sub_ = nh_.subscribe(topic_name, buff_size,
                       &OdometrySubscriber::msg_callback, this);
}

void OdometrySubscriber::msg_callback(
    const nav_msgs::OdometryConstPtr& odom_msg_ptr) {
  PoseData pose_data;
  pose_data.time = odom_msg_ptr->header.stamp.toSec();

  // Set the position
  pose_data.pose(0, 3) = odom_msg_ptr->pose.pose.position.x;
  pose_data.pose(1, 3) = odom_msg_ptr->pose.pose.position.y;
  pose_data.pose(2, 3) = odom_msg_ptr->pose.pose.position.z;

  // Set the orientation
  Eigen::Quaternionf q;
  q.x() = odom_msg_ptr->pose.pose.orientation.x;
  q.y() = odom_msg_ptr->pose.pose.orientation.y;
  q.z() = odom_msg_ptr->pose.pose.orientation.z;
  q.w() = odom_msg_ptr->pose.pose.orientation.w;

  pose_data.pose.block<3, 3>(0, 0) = q.matrix();

  new_pose_data_.push_back(pose_data);
}

void OdometrySubscriber::ParseData(std::deque<PoseData>& deque_pose_data) {
  if (!new_pose_data_.empty()) {
    deque_pose_data.insert(deque_pose_data.end(),
                           new_pose_data_.begin(), new_pose_data_.end());
    new_pose_data_.clear();
  }

}

}  // namespace lidar_slam
