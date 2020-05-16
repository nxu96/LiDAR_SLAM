/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-07 16:55:18
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-16 16:59:57
 * @Description: NDT Based Point Cloud Matching Front End (Lidar Odometry)
 */
#include "front_end/front_end.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <vector>
#include <deque>
#include <boost/filesystem.hpp>
#include "glog/logging.h"
// TODO(nxu): I do not understand this .in header file
#include "global_definition/global_definition.h"

namespace lidar_slam {
FrontEnd::FrontEnd() :
    local_map_ptr_(new CloudData::CLOUD()),
    global_map_ptr_(new CloudData::CLOUD()),
    result_cloud_ptr_(new CloudData::CLOUD()) {
  // NOTE: Set some parameters inside the ctor
  InitWithConfig();
}

bool FrontEnd::InitWithConfig() {
  std::string config_file_path =
    WORK_SPACE_PATH + "/config/front_end/front_config.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  InitDataPath(config_node);
  InitRegistration(regis_ptr_, config_node);
  InitFilter("local_map", local_map_filter_ptr_, config_node);
  InitFilter("frame", frame_filter_ptr_, config_node);
  InitFilter("display", display_filter_ptr_, config_node);

  return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  local_frame_size_ = config_node["local_frame_size"].as<int>();

  return true;
}

bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
  data_path_ = config_node["data_path"].as<std::string>();
  if (data_path_ == "./") {
    data_path_ = WORK_SPACE_PATH;
  }
  data_path_ += "/data/map";
  if (boost::filesystem::is_directory(data_path_)) {
    boost::filesystem::remove_all(data_path_);
  }
  boost::filesystem::create_directory(data_path_);
  if (!boost::filesystem::is_directory(data_path_)) {
    LOG(WARNING) << "Fail to create the directory " << data_path_ << std::endl;
    return false;
  } else {
    LOG(INFO) << "Map point cloud will be saved at " << data_path_ << std::endl;
  }

  std::string key_frame_path = data_path_ + "/key_frames";
  boost::filesystem::create_directory(key_frame_path);
  if (!boost::filesystem::is_directory(key_frame_path)) {
    LOG(WARNING) << "Fail to create the directory " << key_frame_path
                 << std::endl;
    return false;
  } else {
    LOG(INFO) << "Key frames will be saved at " << key_frame_path << std::endl;
  }

  return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>&
  regis_ptr, const YAML::Node& config_node) {
  std::string regis_method =
    config_node["registration_method"].as<std::string>();
  LOG(INFO) << "[Registration Method] " << regis_method << std::endl;
  if (regis_method == "NDT") {
    regis_ptr = std::make_shared<NDTRegistration>(config_node[regis_method]);
  } else {
    LOG(ERROR) << "Registration Method " << regis_method
               << " Not Found" <<std::endl;
    return false;
  }
  return true;
}

bool FrontEnd::InitFilter(std::string filter_user,
  std::shared_ptr<CloudFilterInterface>& filter_ptr,
  const YAML::Node& config_node) {
  std::string filter_method =
    config_node[filter_user+"_filter"].as<std::string>();
  LOG(INFO) << filter_user << " selects the filter method "
            << filter_method << std::endl;
  if (filter_method == "voxel_filter") {
    filter_ptr =
      std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
  } else {
    LOG(ERROR) << "Filtering Method " << filter_method << " for user "
               << filter_user << " Not Found" << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief NOTE: Point cloud matching main function
 * @param cloud_data New point cloud input
 */
bool FrontEnd::Update(const CloudData& cloud_data,
  Eigen::Matrix4f& cloud_pose) {
  current_frame_.cloud_data_.time_ = cloud_data.time_;
  std::vector<int> indices;
  // Remove NaN from the new pcd and save it to the current frame ptr
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_,
  *current_frame_.cloud_data_.cloud_ptr_, indices);

  CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
  frame_filter_ptr_->Filter(current_frame_.cloud_data_.cloud_ptr_,
    filtered_cloud_ptr);

  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;

  // if the local map container is empty, this is the very first frame data
  // now we use this frame as a key frame and update the local and global map
  if (local_map_.empty()) {
    current_frame_.pose_ = init_pose_;
    UpdateNewFrame(current_frame_);
    cloud_pose = current_frame_.pose_;
    return true;
  }

  // This is not the first frame then we perform the normal matching process
  regis_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,
    result_cloud_ptr_, current_frame_.pose_);
  cloud_pose = current_frame_.pose_;

  // Update the transformation between two adjacent frames
  step_pose = last_pose.inverse() * current_frame_.pose_;
  // predict the pose of next step
  predict_pose = current_frame_.pose_ * step_pose;
  last_pose = current_frame_.pose_;
  // Decide whether to add a new keyframe or not, based on Manhattan distance
  if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose_(0, 3)) +
      fabs(last_key_frame_pose(1, 3) - current_frame_.pose_(1, 3)) +
      fabs(last_key_frame_pose(2, 3) - current_frame_.pose_(2, 3)) >
      key_frame_distance_) {
    // NOTE: Add a new keyframe here
    UpdateNewFrame(current_frame_);
    last_key_frame_pose = current_frame_.pose_;
  }

  return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;
  return true;
}

// bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose) {
  // predict_pose_ = predict_pose;
  // return true;
// }

// NOTE: Save a new key frame, think about how pointers are being used here!!
bool FrontEnd::UpdateNewFrame(const Frame& new_key_frame) {
  // save this new key frame candidate to disk to save memory
  std::string file_path = data_path_ + "/key_frames/key_frame_" +
    std::to_string(global_map_.size()) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data_.cloud_ptr_);

  Frame key_frame = new_key_frame;
  // save this point cloud into the dynamic memory
  key_frame.cloud_data_.cloud_ptr_.reset(
      new CloudData::CLOUD(*new_key_frame.cloud_data_.cloud_ptr_));
  CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

  // Update the local map with this new key frame
  local_map_.push_back(key_frame);
  while (local_map_.size() > static_cast<std::size_t>(local_frame_size_)) {
    local_map_.pop_front();
  }
  // NOTE: Everytime we reset and update the local map pointer when we add a
  // new keyframe
  local_map_ptr_.reset(new CloudData::CLOUD());
  for (std::size_t i = 0; i < local_map_.size(); ++i) {
    pcl::transformPointCloud(*local_map_.at(i).cloud_data_.cloud_ptr_,
                              *transformed_cloud_ptr, local_map_.at(i).pose_);
    // Stack the point cloud
    *local_map_ptr_ += *transformed_cloud_ptr;
  }
  // NOTE: Do we need to free this part? No because pcl pointers are all shared
  // delete transformed_cloud_ptr;
  has_new_local_map_ = true;

  // Do not performing filtering if the key frame size is small
  if (local_map_.size() < 10) {
    regis_ptr_->SetInputTarget(local_map_ptr_);
  } else {  // local map is too large, we need to filter it
    CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
    local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
    regis_ptr_->SetInputTarget(filtered_local_map_ptr);
  }

  // Update the global map
  // NOTE: Here we reset the point cloud ptr before save it into the container
  // becasue we have already saved the point cloud to the disk so there
  // is no need to put it into the memory anymore
  key_frame.cloud_data_.cloud_ptr_.reset(new CloudData::CLOUD());
  global_map_.push_back(key_frame);
  return true;
}

bool FrontEnd::SaveMap() {
  global_map_ptr_.reset(new CloudData::CLOUD());
  // NOTE: Load all the key frames first
  std::string key_frame_path = "";
  CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
  CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
  for (std::size_t i = 0; i < global_map_.size(); ++i) {
    key_frame_path = data_path_ + "/key_frames/key_frame_"
      + std::to_string(i) + ".pcd";
    pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);
    pcl::transformPointCloud(*key_frame_cloud_ptr, *transformed_cloud_ptr,
      global_map_.at(i).pose_);
    *global_map_ptr_ += *transformed_cloud_ptr;
  }
  std::string map_file_path = data_path_ + "/map.pcd";
  pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
  has_new_global_map_ = true;
  return true;
}

// these three functions are mainly used for visualization I guess
bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
  if (has_new_local_map_) {
    display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
    return true;
  }
  return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
  if (has_new_global_map_) {
    has_new_global_map_ = false;
    display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    global_map_ptr_.reset(new CloudData::CLOUD());
    return true;
  }
  return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
  display_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);
  return true;
}

}  // namespace lidar_slam
