/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 14:27:29
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 14:29:54
 * @Description: Pose of IMU/GNSS after processing
 */
#ifndef LIDAR_SLAM_INCLUDE_SENSOR_DATA_POSE_DATA_H_
#define LIDAR_SLAM_INCLUDE_SENSOR_DATA_POSE_DATA_H_
#include <Eigen/Dense>
namespace lidar_slam {
class PoseData {
 public:
  double time = 0.0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

 public:
  Eigen::Quaternionf GetQuaternion();
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SENSOR_DATA_POSE_DATA_H_
