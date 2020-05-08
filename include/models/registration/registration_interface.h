/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 11:35:55
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 11:51:36
 * @Description: Point cloud registration interface base class
 */

#ifndef LIDAR_SLAM_INCLUDE_MODELS_REGISTRATION_REGISTRATION_INTERFACE_H_
#define LIDAR_SLAM_INCLUDE_MODELS_REGISTRATION_REGISTRATION_INTERFACE_H_
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "sensor_data/cloud_data.h"
namespace lidar_slam {
class RegistrationInterface {
 public:
  // Notice here we used a default virtual destructor in the base class
  virtual ~RegistrationInterface() = default;

  virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;

  virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
    const Eigen::Matrix4f& predict_pose, CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose) = 0;
};
}  // namespace lidar_slam


#endif  // LIDAR_SLAM_INCLUDE_MODELS_REGISTRATION_REGISTRATION_INTERFACE_H_
