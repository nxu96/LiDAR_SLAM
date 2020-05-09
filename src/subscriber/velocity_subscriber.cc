/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-09 10:31:44
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-09 13:40:10
 * @Description: Velocity subscriber implementation
 */

#include "subscriber/velocity_subscriber.h"
#include "glog/logging.h"

namespace lidar_slam {
VelocitySubscriber::VelocitySubscriber(const ros::NodeHandle& nh,
  std::string topic_name, std::size_t buff_size) : nh_(nh) {
  sub_ = nh_.subscribe(topic_name, buff_size,
    &VelocitySubscriber::OnVelocity, this);
}

void VelocitySubscriber::ParseData(
  std::deque<VelocityData>* deque_velocity_data) {
  if (!new_velocity_data_.empty()) {
    deque_velocity_data->insert(deque_velocity_data->end(),
      new_velocity_data_.begin(), new_velocity_data_.end());
    new_velocity_data_.clear();
  }
}

void VelocitySubscriber::OnVelocity(
  const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr) {
  VelocityData data;
  data.time_ = twist_msg_ptr->header.stamp.toSec();

  data.linear_velocity_.x_ = twist_msg_ptr->twist.linear.x;
  data.linear_velocity_.y_ = twist_msg_ptr->twist.linear.y;
  data.linear_velocity_.z_ = twist_msg_ptr->twist.linear.z;

  data.angular_velocity_.x_ = twist_msg_ptr->twist.angular.x;
  data.angular_velocity_.y_ = twist_msg_ptr->twist.angular.y;
  data.angular_velocity_.z_ = twist_msg_ptr->twist.angular.z;

  new_velocity_data_.push_back(data);
}

}  // namespace lidar_slam
