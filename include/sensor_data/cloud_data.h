/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 11:33:55
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 12:29:12
 * @Description: Description
 */
#ifndef LIDAR_SLAM_INCLUDE_SENSOR_DATA_CLOUD_DATA_H_
#define LIDAR_SLAM_INCLUDE_SENSOR_DATA_CLOUD_DATA_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_slam {
class CloudData {
 public:
  using POINT = pcl::PointXYZ;
  using CLOUD = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

 public:
  CloudData() : cloud_ptr_(new CLOUD()) {}

 public:
  double time_ = 0.0;
  CLOUD_PTR cloud_ptr_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_SENSOR_DATA_CLOUD_DATA_H_
