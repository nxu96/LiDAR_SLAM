/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-07 16:55:18
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-07 23:37:16
 * @Description: NDT Based Point Cloud Matching Front End (Lidar Odometry)
 */
#include "front_end/front_end.h"
#include <pcl/common/transforms.h>
#include <cmath>
#include <vector>
#include "glog/logging.h"
namespace lidar_slam {
FrontEnd::FrontEnd() :
    ndt_ptr_(
      new pcl::NormalDistributionsTransform
      <CloudData::POINT, CloudData::POINT>()),
    local_map_ptr_(new CloudData::CLOUD()),
    global_map_ptr_(new CloudData::CLOUD()),
    result_cloud_ptr_(new CloudData::CLOUD()) {
  // NOTE: Set some parameters inside the ctor
  cloud_filter_.setLeafSize(1.3, 1.3, 1.3);
  local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
  display_filter_.setLeafSize(0.5, 0.5, 0.5);
  ndt_ptr_->setResolution(1.0);
  ndt_ptr_->setStepSize(0.1);
  ndt_ptr_->setTransformationEpsilon(0.01);
  ndt_ptr_->setMaximumIterations(30);
}

// NOTE: Point Cloud Matching main function
Eigen::Matrix4f FrontEnd::Update(const CloudData& cloud_data) {
  current_frame_.cloud_data_.time_ = cloud_data.time_;
  std::vector<int> indices;
  // Remove NaN from the new pcd and save it to the current frame ptr
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_,
  *current_frame_.cloud_data_.cloud_ptr_, indices);

  CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
  cloud_filter_.setInputCloud(current_frame_.cloud_data_.cloud_ptr_);
  cloud_filter_.filter(*filtered_cloud_ptr);

  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose_ = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;

  // if the local map container is empty, this is the very first frame data
  // now we use this frame as a key frame and update the local and global map
  if (local_map_.empty()) {
    current_frame_.pose_ = init_pose_;
    UpdateNewFrame(current_frame_);
    return current_frame_.pose_;
  }

  // This is not the first frame then we perform the normal matching process
  ndt_ptr_->setInputSource(filtered_cloud_ptr);
  // predict pose provides an initial guess to reduce the matching time
  ndt_ptr_->align(*result_cloud_ptr_, predict_pose_);
  current_frame_.pose_ = ndt_ptr_->getFinalTransformation();
  // Update the transformation between two adjacent frames
  step_pose = last_pose.inverse() * current_frame_.pose_;
  // predict the pose of next step
  predict_pose_ = current_frame_.pose_ * step_pose;
  last_pose = current_frame_.pose_;
  // Decide whether to add a new keyframe or not, based on Manhattan distance
  const float manhattan_dist_thresh = 2.0;
  if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose_(0, 3)) +
      fabs(last_key_frame_pose(1, 3) - current_frame_.pose_(1, 3)) +
      fabs(last_key_frame_pose(2, 3) - current_frame_.pose_(2, 3)) >
      manhattan_dist_thresh) {
    UpdateNewFrame(current_frame_);
    last_key_frame_pose = current_frame_.pose_;
  }

  return current_frame_.pose_;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;
  return true;
}

bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose) {
  predict_pose_ = predict_pose;
  return true;
}

// NOTE: Save a new key frame, think about how pointers are being used here!!
void FrontEnd::UpdateNewFrame(const Frame& new_key_frame) {
  // save this new key frame candidate
  Frame key_frame = new_key_frame;
  // save this point cloud into the dynamic memory
  key_frame.cloud_data_.cloud_ptr_.reset(
      new CloudData::CLOUD(*new_key_frame.cloud_data_.cloud_ptr_));
  CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

  // Update the local map with this new key frame
  local_map_.push_back(key_frame);
  while (local_map_.size() > 20) {
    local_map_.pop_front();
  }
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

  if (local_map_.size() < 10) {
    ndt_ptr_->setInputTarget(local_map_ptr_);
  } else {  // local map is too large, we need to filter it
    CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
    local_map_filter_.setInputCloud(local_map_ptr_);
    local_map_filter_.filter(*filtered_local_map_ptr);
    ndt_ptr_->setInputTarget(filtered_local_map_ptr);
  }

  // Update the global map
  global_map_.push_back(key_frame);
  if (global_map_.size() % 100 != 0) {
    return;
  } else {
    global_map_ptr_.reset(new CloudData::CLOUD());
    for (std::size_t i = 0; i < global_map_.size(); ++i) {
      pcl::transformPointCloud(*global_map_.at(i).cloud_data_.cloud_ptr_,
                               *transformed_cloud_ptr, global_map_.at(i).pose_);
      *global_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_global_map_ = true;
  }
}
// these three functions are mainly used for visualization I guess
bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
  if (has_new_local_map_) {
    display_filter_.setInputCloud(local_map_ptr_);
    display_filter_.filter(*local_map_ptr);
    return true;
  }
  return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
  if (has_new_global_map_) {
    display_filter_.setInputCloud(global_map_ptr_);
    display_filter_.filter(*global_map_ptr);
    return true;
  }
  return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
  display_filter_.setInputCloud(result_cloud_ptr_);
  display_filter_.filter(*current_scan_ptr);
  return true;
}

}  // namespace lidar_slam
