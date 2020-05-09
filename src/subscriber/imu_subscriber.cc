/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 17:51:27
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-09 10:29:00
 * @Description: Description
 */

#include "subscriber/imu_subscriber.h"

namespace lidar_slam {
IMUSubscriber::IMUSubscriber(const ros::NodeHandle& nh, std::string topic,
    size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic, buff_size, &IMUSubscriber::OnIMU, this);
}

void IMUSubscriber::ParseData(std::deque<IMUData>* deque_imu_data) {
  if (!new_imu_data_.empty()) {
    deque_imu_data->insert(deque_imu_data->end(), new_imu_data_.begin(),
                           new_imu_data_.end());
    new_imu_data_.clear();
  }
}

void IMUSubscriber::OnIMU(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
  IMUData imu_data;
  imu_data.time_ = imu_msg_ptr->header.stamp.toSec();

  imu_data.linear_acceleration_.x = imu_msg_ptr->linear_acceleration.x;
  imu_data.linear_acceleration_.y = imu_msg_ptr->linear_acceleration.y;
  imu_data.linear_acceleration_.z = imu_msg_ptr->linear_acceleration.z;

  imu_data.angular_velocity_.x = imu_msg_ptr->angular_velocity.x;
  imu_data.angular_velocity_.y = imu_msg_ptr->angular_velocity.y;
  imu_data.angular_velocity_.z = imu_msg_ptr->angular_velocity.z;

  imu_data.orientation_.x = imu_msg_ptr->orientation.x;
  imu_data.orientation_.y = imu_msg_ptr->orientation.y;
  imu_data.orientation_.z = imu_msg_ptr->orientation.z;
  imu_data.orientation_.w = imu_msg_ptr->orientation.w;

  new_imu_data_.push_back(imu_data);
}

}  // namespace lidar_slam
