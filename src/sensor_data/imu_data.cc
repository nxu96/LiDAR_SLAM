/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-09 11:34:35
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-09 13:45:33
 * @Description: Description
 */
#include "sensor_data/imu_data.h"

namespace lidar_slam {
Eigen::Matrix3f IMUData::GetOrientationMatrix() {
  Eigen::Quaterniond q(orientation_.w,
                orientation_.x, orientation_.y, orientation_.z);
  Eigen::Matrix3f matrix = q.matrix().cast<float>();
  return matrix;
}

bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData,
    std::deque<IMUData>& SyncedData, double sync_time) {
  while (UnsyncedData.size() >= 2) {
    if (UnsyncedData.front().time_ > sync_time) {
      return false;
    }
    if (UnsyncedData.back().time_ < sync_time) {
      return false;
    }
    if (UnsyncedData.at(1).time_ < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }

    // huge time diff ----> lost data
    if (sync_time - UnsyncedData.front().time_ > 0.2) {
      UnsyncedData.pop_front();
      // break;
      return false;
    }
    if (UnsyncedData.at(1).time_ - sync_time > 0.2) {
      // UnsyncedData.pop_front();
      // break;
      return false;
    }

    break;
  }
  if (UnsyncedData.size() < 2) {
    return false;
  }
  IMUData front_data = UnsyncedData.at(0);
  IMUData back_data = UnsyncedData.at(1);
  IMUData synced_data;

  double front_scale = (back_data.time_ - sync_time) /
    (back_data.time_ - front_data.time_);
  double back_scale = (sync_time - front_data.time_) /
    (back_data.time_ - front_data.time_);

  synced_data.time_ = sync_time;
  synced_data.linear_acceleration_.x =
                                front_data.linear_acceleration_.x * front_scale
                              + back_data.linear_acceleration_.x * back_scale;
  synced_data.linear_acceleration_.y =
                                front_data.linear_acceleration_.y * front_scale
                              + back_data.linear_acceleration_.y * back_scale;
  synced_data.linear_acceleration_.z =
                                front_data.linear_acceleration_.z * front_scale
                              + back_data.linear_acceleration_.z * back_scale;
  synced_data.angular_velocity_.x =
                                front_data.angular_velocity_.x * front_scale +
                                back_data.angular_velocity_.x * back_scale;
  synced_data.angular_velocity_.y =
                                front_data.angular_velocity_.y * front_scale +
                                back_data.angular_velocity_.y * back_scale;
  synced_data.angular_velocity_.z =
                                front_data.angular_velocity_.z * front_scale +
                                back_data.angular_velocity_.z * back_scale;
  // quaternion interpolation : linear/spherical
  synced_data.orientation_.x =  front_data.orientation_.x * front_scale +
                                back_data.orientation_.x * back_scale;
  synced_data.orientation_.y =  front_data.orientation_.y * front_scale +
                                back_data.orientation_.y * back_scale;
  synced_data.orientation_.z =  front_data.orientation_.z * front_scale +
                                back_data.orientation_.z * back_scale;
  synced_data.orientation_.w =  front_data.orientation_.w * front_scale +
                                back_data.orientation_.w * back_scale;

  synced_data.orientation_.Normalize();

  SyncedData.push_back(synced_data);

  return true;
}
}  // namespace lidar_slam
