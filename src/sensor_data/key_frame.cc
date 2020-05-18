/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 17:17:05
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 12:27:43
 * @Description: Key frame class implementation
 */
#include "sensor_data/key_frame.h"
#include <Eigen/Dense>

namespace lidar_slam {
Eigen::Quaternionf KeyFrame::GetQuaternion() {
  // NOTE: Cpplint: No reduant blank lines at the begining or the end of a
  // scope/function
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));

  return q;
}
}  // namespace lidar_slam
