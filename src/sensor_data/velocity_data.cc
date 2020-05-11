/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-09 10:34:19
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-10 18:36:08
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
}  // namespace lidar_slam
