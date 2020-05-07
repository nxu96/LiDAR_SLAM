/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 00:19:51
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 18:42:51
 * @Description: Publish Point Cloud
 */

#ifndef LIDAR_SLAM_INCLUDE_PUBLISHER_CLOUD_PUBLISHER_H_
#define LIDAR_SLAM_INCLUDE_PUBLISHER_CLOUD_PUBLISHER_H_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include "sensor_data/cloud_data.h"

namespace lidar_slam {
class CloudPublisher {
 public:
  CloudPublisher(const ros::NodeHandle& nh, std::string topic_name,
                 size_t buff_size, std::string frame_id);
  CloudPublisher() = default;
  void Publish(CloudData::CLOUD_PTR cloud_ptr_input);

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string frame_id_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_PUBLISHER_CLOUD_PUBLISHER_H_
