/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-16 21:40:43
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 11:41:00
 * @Description: Front end flow management
 */
#ifndef LIDAR_SLAM_INCLUDE_MAPPING_FRONT_END_FRONT_END_FLOW_H_
#define LIDAR_SLAM_INCLUDE_MAPPING_FRONT_END_FRONT_END_FLOW_H_
#include <ros/ros.h>
#include <memory>
#include <deque>
#include "subscriber/cloud_subscriber.h"
#include "publisher/odometry_publisher.h"
#include "mapping/front_end/front_end.h"

namespace lidar_slam {
class FrontEndFlow {
 public:
  explicit FrontEndFlow(ros::NodeHandle& nh);

  bool Run();
 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateLidarOdometry();
  bool PublishData();
 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometryPublisher> lidar_odom_pub_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  CloudData current_cloud_data_;
  Eigen::Matrix4f lidar_odometry_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_MAPPING_FRONT_END_FRONT_END_FLOW_H_

