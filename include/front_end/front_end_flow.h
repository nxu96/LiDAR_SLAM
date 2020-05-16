/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 12:42:34
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-16 16:43:49
 * @Description: Front end flow header file
 */
#ifndef LIDAR_SLAM_INCLUDE_FRONT_END_FRONT_END_FLOW_H_
#define LIDAR_SLAM_INCLUDE_FRONT_END_FRONT_END_FLOW_H_
#include <ros/ros.h>
#include <memory>
#include <deque>
// subscriber
#include "subscriber/cloud_subscriber.h"
#include "subscriber/gnss_subscriber.h"
#include "subscriber/imu_subscriber.h"
#include "subscriber/velocity_subscriber.h"
// publisher
#include "publisher/cloud_publisher.h"
#include "publisher/odometry_publisher.h"
// listener
#include "tf_listener/tf_listener.h"
// front end class
#include "front_end/front_end.h"
// distortion adjust
#include "models/scan_adjust/distortion_adjust.h"
namespace lidar_slam {
class FrontEndFlow {
 public:
  explicit FrontEndFlow(ros::NodeHandle& nh);

  bool Run();
  bool SaveMap();
  bool PublishGlobalMap();

 private:
  bool ReadData();
  bool Initcalibration();
  bool InitGNSS();
  bool hasData();
  bool ValidData();
  bool UpdateGNSSOdometry();
  bool UpdateLidarOdometry();
  bool PublishData();
  bool SaveTrajectory();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
  std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
  std::shared_ptr<TFListener> lidar_to_imu_ptr_;
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<OdometryPublisher> lidar_odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> gnss_odom_pub_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;
  std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;
  std::deque<CloudData> cloud_data_buff_;
  std::deque<IMUData> imu_data_buff_;
  std::deque<GNSSData> gnss_data_buff_;
  std::deque<VelocityData> velocity_data_buff_;
  Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
  CloudData current_cloud_data_;
  IMUData current_imu_data_;
  GNSSData current_gnss_data_;
  VelocityData current_velocity_data_;

  CloudData::CLOUD_PTR local_map_ptr_;
  CloudData::CLOUD_PTR global_map_ptr_;
  CloudData::CLOUD_PTR current_scan_ptr_;
  Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f lidar_odometry_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_FRONT_END_FRONT_END_FLOW_H_
