/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-07 17:03:23
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 15:46:19
 * @Description: Front end module header file
 */

#ifndef LIDAR_SLAM_INCLUDE_FRONT_END_FRONT_END_H_
#define LIDAR_SLAM_INCLUDE_FRONT_END_FRONT_END_H_

#include <Eigen/Dense>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>
#include <deque>  // sliding window needs this
#include <memory>
#include <string>
#include "models/registration/ndt_registration.h"
#include "models/cloud_filter/voxel_filter.h"
#include "sensor_data/cloud_data.h"

namespace lidar_slam {
class FrontEnd {
 public:
  struct Frame {
    Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
    CloudData cloud_data_;
  };

 public:
  FrontEnd();
  bool InitWithConfig();
  // Take the latest point cloud and find the match, compute the transform
  bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
  bool SetInitPose(const Eigen::Matrix4f& init_pose);
  // bool SetPredictPose(const Eigen::Matrix4f& predict_pose);
  bool SaveMap();
  bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
  bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
  bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

 private:
  bool InitParam(const YAML::Node& config_node);
  bool InitDataPath(const YAML::Node& config_node);
  bool InitRegistration(std::shared_ptr<RegistrationInterface>& regis_ptr,
    const YAML::Node& config_node);
  bool InitFilter(std::string filter_user,
    std::shared_ptr<CloudFilterInterface>& filter_ptr,
    const YAML::Node& config_node);
  bool UpdateNewFrame(const Frame& new_key_frame);


 private:
  std::string data_path_ = "";
  // filters
  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> display_filter_ptr_;
  // registration
  std::shared_ptr<RegistrationInterface> regis_ptr_;
  // map container
  std::deque<Frame> local_map_;
  std::deque<Frame> global_map_;

  bool has_new_local_map_ = false;
  bool has_new_global_map_ = false;
  CloudData::CLOUD_PTR local_map_ptr_;
  CloudData::CLOUD_PTR global_map_ptr_;
  CloudData::CLOUD_PTR result_cloud_ptr_;
  Frame current_frame_;
  // TODO(nxu): asd
  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

  float key_frame_distance_ = 2.0;
  int local_frame_size_ = 20;
  // Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_FRONT_END_FRONT_END_H_
