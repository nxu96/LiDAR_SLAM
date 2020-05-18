/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 17:22:32
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 17:25:48
 * @Description: Pose data class implementation
 */
#include "sensor_data/pose_data.h"

namespace lidar_slam {
Eigen::Quaternionf PoseData::GetQuaternion() {
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));

  return q;
}

}  // namespace lidar_slam
