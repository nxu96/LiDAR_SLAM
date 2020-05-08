/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 10:41:14
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 11:10:54
 * @Description: Interface for point cloud filtering module
 */

#ifndef LIDAR_SLAM_INCLUDE_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_H_
#define LIDAR_SLAM_INCLUDE_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_H_
#include <yaml-cpp/yaml.h>
#include "sensor_data/cloud_data.h"

namespace lidar_slam {
class CloudFilterInterface {
 public:
  virtual ~CloudFilterInterface() = default;
  // pure virtural function
  virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
    CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};

}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_H_
