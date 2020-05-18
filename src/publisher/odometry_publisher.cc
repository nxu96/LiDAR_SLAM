/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 19:13:24
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 17:13:22
 * @Description: Description
 */

#include "publisher/odometry_publisher.h"

namespace lidar_slam {
OdometryPublisher::OdometryPublisher(const ros::NodeHandle& nh,
    std::string topic_name, std::string base_frame_id,
    std::string child_frame_id, size_t buff_size) : nh_(nh) {
  pub_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
  odometry_.header.frame_id = base_frame_id;
  odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix,
                                double time) {
  ros::Time ros_time(static_cast<float>(time));
  PublishData(transform_matrix, ros_time);
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
  ros::Time ros_time = ros::Time::now();
  PublishData(transform_matrix, ros_time);
}

void OdometryPublisher::PublishData(const Eigen::Matrix4f& transform_matrix,
                                    ros::Time ros_time) {
  odometry_.header.stamp = ros_time;

  // set the position
  odometry_.pose.pose.position.x = transform_matrix(0, 3);
  odometry_.pose.pose.position.y = transform_matrix(1, 3);
  odometry_.pose.pose.position.z = transform_matrix(2, 3);

  // set the orientation from the quternion
  Eigen::Quaternionf q(transform_matrix.block<3, 3>(0, 0));
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  pub_.publish(odometry_);
}

bool OdometryPublisher::HasSubscribers() {
  return pub_.getNumSubscribers() != 0;
}
}  // namespace lidar_slam
