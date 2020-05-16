/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-14 22:27:28
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-14 22:49:29
 * @Description: Point Cloud Distortion Motion Compensation
 */
#ifndef LIDAR_SLAM_INCLUDE_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_
#define LIDAR_SLAM_INCLUDE_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "sensor_data/velocity_data.h"
#include "sensor_data/cloud_data.h"

namespace lidar_slam {
class DistortionAdjust {
 public:
  void SetMotionInfo(float scan_period, VelocityData velo_data);

  bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr,
                   CloudData::CLOUD_PTR& output_cloud_ptr);

 private:
  inline Eigen::Matrix3f UpdateMatrix(float real_time);

 private:
  float scan_period_;
  Eigen::Vector3f velocity_;
  Eigen::Vector3f angular_rate_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_
