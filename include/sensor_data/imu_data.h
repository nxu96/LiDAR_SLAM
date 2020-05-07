/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 12:22:06
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 12:30:25
 * @Description: IMUData class
 */

#ifndef LIDAR_SLAM_INCLUDE_SENSOR_DATA_IMU_DATA_H_
#define LIDAR_SLAM_INCLUDE_SENSOR_DATA_IMU_DATA_H_

#include <Eigen/Dense>

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

  struct Orientation {
    // this is a quaternion representation [x, y, z, w];
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;
  };
  double time_ = 0.0;
  LinearAcceleration linear_acceleration_;
  AngularVelocity angular_velocity_;
  Orientation orientation_;

 public:
  // Convert the quaternion to 3x3 rotation matrix
  Eigen::Matrix3f GetOrientationMatrix() {
    Eigen::Quaterniond q(orientation_.w,
                        orientation_.x, orientation_.y, orientation_.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();
    return matrix;
  }
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_SENSOR_DATA_IMU_DATA_H_
