/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 14:20:49
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 17:25:37
 * @Description: Keyframe Data Definition header file
 */
#ifndef LIDAR_SLAM_INCLUDE_SENSOR_DATA_KEY_FRAME_H_
#define LIDAR_SLAM_INCLUDE_SENSOR_DATA_KEY_FRAME_H_
#include <Eigen/Dense>
namespace lidar_slam {
class KeyFrame {
 public:
  double time = 0.0;
  // NOTE: KeyFrame is basically the same as the PoseData,
  // with an additional idx
  // TODO(nxu): What is this index used for?
  unsigned int index = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

 public:
  Eigen::Quaternionf GetQuaternion();

};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SENSOR_DATA_KEY_FRAME_H_
