/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 12:09:47
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 12:41:19
 * @Description: NDT registration method implementation
 */
#include "models/registration/ndt_registration.h"
#include "glog/logging.h"

namespace lidar_slam {
NDTRegistration::NDTRegistration(const YAML::Node& node) :
  ndt_ptr_(new pcl::NormalDistributionsTransform
  <CloudData::POINT, CloudData::POINT>()) {
  float res = node["res"].as<float>();
  float step_size = node["step_size"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  float max_iter = node["max_iter"].as<int>();
  SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps,
  int max_iter) :
  ndt_ptr_(new pcl::NormalDistributionsTransform
  <CloudData::POINT, CloudData::POINT>()) {
  SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
  ndt_ptr_->setInputTarget(input_target);
  return true;
}

bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
    const Eigen::Matrix4f& predict_pose, CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose) {
  ndt_ptr_->setInputSource(input_source);
  ndt_ptr_->align(*result_cloud_ptr, predict_pose);
  result_pose = ndt_ptr_->getFinalTransformation();
  return true;
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size,
  float trans_eps, int max_iter) {
  ndt_ptr_->setResolution(res);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setTransformationEpsilon(trans_eps);
  ndt_ptr_->setMaximumIterations(max_iter);

  LOG(INFO) << "NDT Params：" << std::endl
            << "res: " << res << ", "
            << "step_size: " << step_size << ", "
            << "trans_eps: " << trans_eps << ", "
            << "max_iter: " << max_iter
            << std::endl;
  return true;
}
}  // namespace lidar_slam
