/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 10:41:52
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 12:15:58
 * @Description: NDT registration method header file
 */

#ifndef LIDAR_SLAM_INCLUDE_MODELS_REGISTRATION_NDT_REGISTRATION_H_
#define LIDAR_SLAM_INCLUDE_MODELS_REGISTRATION_NDT_REGISTRATION_H_
#include <pcl/registration/ndt.h>
#include "models/registration/registration_interface.h"
namespace lidar_slam {
class NDTRegistration : public RegistrationInterface {
 public:
  explicit NDTRegistration(const YAML::Node& node);
  NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

  bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;

  bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
    const Eigen::Matrix4f& predict_pose, CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose) override;

 private:
  bool SetRegistrationParam(float res, float step_size, float trans_eps,
    int max_iter);

 private:
  pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr
    ndt_ptr_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_MODELS_REGISTRATION_NDT_REGISTRATION_H_
