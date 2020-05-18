/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 12:42:34
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 21:31:05
 * @Description: Data preprocess module including time stamp synchronization and
 *               point cloud distortion adjustment etc.
 */
#ifndef LIDAR_SLAM_INCLUDE_PRE_PROCESS_PRE_PROCESS_FLOW_H_
#define LIDAR_SLAM_INCLUDE_PRE_PROCESS_PRE_PROCESS_FLOW_H_
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
// distortion adjust
#include "models/scan_adjust/distortion_adjust.h"

namespace lidar_slam {
class PreProcessFlow {
 public:
  explicit PreProcessFlow(ros::NodeHandle& nh);

  bool Run();

 private:
  bool ReadData();
  bool Initcalibration();
  bool InitGNSS();
  bool HasData();
  bool ValidData();
  bool TransFormData();
  bool PublishData();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
  std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
  std::shared_ptr<TFListener> lidar_to_imu_ptr_;

  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<OdometryPublisher> gnss_odom_pub_ptr_;
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

  Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_PRE_PROCESS_PRE_PROCESS_FLOW_H_
