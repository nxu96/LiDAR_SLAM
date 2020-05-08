/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 10:35:12
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 11:52:05
 * @Description: Voxel filter header file
 */
#ifndef LIDAR_SLAM_INCLUDE_MODELS_CLOUD_FILTER_VOXEL_FILTER_H_
#define LIDAR_SLAM_INCLUDE_MODELS_CLOUD_FILTER_VOXEL_FILTER_H_
#include <pcl/filters/voxel_grid.h>
#include "models/cloud_filter/cloud_filter_interface.h"

namespace lidar_slam {
class VoxelFilter : public CloudFilterInterface {
 public:
  explicit VoxelFilter(const YAML::Node& node);

  VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
    CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

 private:
  bool SetFilterParam(float leaf_size_x, float leaf_size_y,
    float leaf_size_z);

 private:
  pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_MODELS_CLOUD_FILTER_VOXEL_FILTER_H_
