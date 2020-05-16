/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-09 10:34:19
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-16 16:29:58
 * @Description: Velocity data class implementation
 */

#include <iostream>
#include "sensor_data/velocity_data.h"
#include "glog/logging.h"

namespace lidar_slam {
bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData,
  std::deque<VelocityData>& SyncedData, double sync_time) {
  // find the proper data for this sync time (point cloud time)
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
  VelocityData front_data = UnsyncedData.at(0);
  VelocityData back_data = UnsyncedData.at(1);
  VelocityData synced_data;

  double front_scale = (back_data.time_ - sync_time) /
    (back_data.time_ - front_data.time_);
  double back_scale = (sync_time - front_data.time_) /
    (back_data.time_ - front_data.time_);

  synced_data.time_ = sync_time;
  synced_data.linear_velocity_.x_ = front_data.linear_velocity_.x_ * front_scale
                                   + back_data.linear_velocity_.x_ * back_scale;
  synced_data.linear_velocity_.y_ = front_data.linear_velocity_.y_ * front_scale
                                   + back_data.linear_velocity_.y_ * back_scale;
  synced_data.linear_velocity_.z_ = front_data.linear_velocity_.z_ * front_scale
                                   + back_data.linear_velocity_.z_ * back_scale;
  synced_data.angular_velocity_.x_ =
                                front_data.angular_velocity_.x_ * front_scale +
                                back_data.angular_velocity_.x_ * back_scale;
  synced_data.angular_velocity_.y_ =
                                front_data.angular_velocity_.y_ * front_scale +
                                back_data.angular_velocity_.y_ * back_scale;
  synced_data.angular_velocity_.z_ =
                                front_data.angular_velocity_.z_ * front_scale +
                                back_data.angular_velocity_.z_ * back_scale;
  SyncedData.push_back(synced_data);
  return true;
}

void VelocityData::TransformCoordinate(Eigen::Matrix4f imu_to_lidar) {
  Eigen::Matrix4d matrix = imu_to_lidar.cast<double>();
  Eigen::Matrix3d rot_mat = matrix.block<3, 3>(0, 0);
  Eigen::Vector3d w(angular_velocity_.x_, angular_velocity_.y_,
                    angular_velocity_.z_);
  Eigen::Vector3d v(linear_velocity_.x_, linear_velocity_.y_,
                    linear_velocity_.z_);
  w = rot_mat * w;
  v = rot_mat * v;
  Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
  Eigen::Vector3d delta_v;
  // TODO(nxu): Figure out the IMU model here
  delta_v(0) = w(1) * r(2) - w(2) * r(1);
  delta_v(1) = w(2) * r(0) - w(0) * r(2);
  delta_v(2) = w(1) * r(1) - w(1) * r(0);
  v = v + delta_v;
  angular_velocity_.x_ = w(0);
  angular_velocity_.y_ = w(1);
  angular_velocity_.z_ = w(2);
  linear_velocity_.x_ = v(0);
  linear_velocity_.y_ = v(1);
  linear_velocity_.z_ = v(2);
}
}  // namespace lidar_slam
