/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 12:22:06
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-09 11:41:57
 * @Description: IMUData class
 */

#ifndef LIDAR_SLAM_INCLUDE_SENSOR_DATA_IMU_DATA_H_
#define LIDAR_SLAM_INCLUDE_SENSOR_DATA_IMU_DATA_H_

#include <Eigen/Dense>
#include <deque>
#include <string>
#include <cmath>
namespace lidar_slam {
class IMUData {
 public:
  struct LinearAcceleration {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct AngularVelocity {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  class Orientation {
   public:
    // this is a quaternion representation [x, y, z, w];
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;

   public:
    void Normalize() {
      double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
      x /= norm;
      y /= norm;
      z /= norm;
      w /= norm;
    }
  };
  double time_ = 0.0;
  LinearAcceleration linear_acceleration_;
  AngularVelocity angular_velocity_;
  Orientation orientation_;

 public:
  // Convert the quaternion to 3x3 rotation matrix
  Eigen::Matrix3f GetOrientationMatrix();
  static bool SyncData(std::deque<IMUData>& UnsyncedData,
    std::deque<IMUData>& SyncedData, double sync_time);
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_SENSOR_DATA_IMU_DATA_H_
