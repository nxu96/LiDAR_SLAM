/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 11:50:26
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 12:05:14
 * @Description: Voxel filter source file
 */
#include "models/cloud_filter/voxel_filter.h"
#include "glog/logging.h"

namespace lidar_slam {
VoxelFilter::VoxelFilter(const YAML::Node& node) {
  float leaf_size_x = node["leaf_size"][0].as<float>();
  float leaf_size_y = node["leaf_size"][1].as<float>();
  float leaf_size_z = node["leaf_size"][2].as<float>();
  SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y,
  float leaf_size_z) {
  SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
  CloudData::CLOUD_PTR& filtered_cloud_ptr) {
  voxel_filter_.setInputCloud(input_cloud_ptr);
  voxel_filter_.filter(*filtered_cloud_ptr);

  return true;
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y,
  float leaf_size_z) {
  voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
  // logging info here
  LOG(INFO) << "Voxel Filter Params: " << std::endl
            << leaf_size_x <<", " << leaf_size_y << ", "
            << leaf_size_z << std::endl;

  return true;
}
}  // namespace lidar_slam
