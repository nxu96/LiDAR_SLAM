/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-09 10:03:40
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-09 10:05:23
 * @Description: Velocity data header file
 */
#ifndef LIDAR_SLAM_INCLUDE_SENSOR_DATA_VELOCITY_DATA_H_
#define LIDAR_SLAM_INCLUDE_SENSOR_DATA_VELOCITY_DATA_H_

#include <deque>

namespace lidar_slam {
class VelocityData {
 public:
  struct LinearVelocity {
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
  };

  struct AngularVelocity {
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
  };

  double time_ = 0.0;
  LinearVelocity linear_velocity_;
  AngularVelocity angular_velocity_;

 public:
  // how is this static function being used?
  static bool SyncData(std::deque<VelocityData>& UnsyncedData,
    std::deque<VelocityData>& SyncedData, double sync_time);
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SENSOR_DATA_VELOCITY_DATA_H_
