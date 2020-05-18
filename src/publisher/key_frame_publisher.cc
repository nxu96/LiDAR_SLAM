/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 16:38:06
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 11:52:28
 * @Description: Single Key frame publisher implementation
 */
#include "publisher/key_frame_publisher.h"
#include <Eigen/Dense>
namespace lidar_slam {
KeyFramePublisher::KeyFramePublisher(ros::NodeHandle& nh,
                                     std::string topic_name,
                                     std::string frame_id, size_t buff_size) :
    nh_(nh), frame_id_(frame_id) {
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name, buff_size);
}

void KeyFramePublisher::Publish(KeyFrame& key_frame) {
  geometry_msgs::PoseStamped pose;
  ros::Time ros_time(static_cast<float>(key_frame.time));
  pose.header.stamp = ros_time;
  pose.header.frame_id = frame_id_;
  pose.header.seq = key_frame.index;
  pose.pose.position.x = key_frame.pose(0, 3);
  pose.pose.position.y = key_frame.pose(1, 3);
  pose.pose.position.z = key_frame.pose(2, 3);

  Eigen::Quaternionf q = key_frame.GetQuaternion();
  pose.pose.orientation.w = q.w();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();

  pub_.publish(pose);
}

bool KeyFramePublisher::HasSubscribers() {
  return pub_.getNumSubscribers() != 0;
}

}  // namespace lidar_slam
