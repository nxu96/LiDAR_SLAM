/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 22:34:21
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 22:47:58
 * @Description: Back End flow header file
 */
#ifndef LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_FLOW_H_
#define LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_FLOW_H_

#include <ros/ros.h>
#include <deque>
#include <memory>
#include "subscriber/cloud_subscriber.h"
#include "subscriber/odometry_subscriber.h"
#include "publisher/odometry_publisher.h"
#include "publisher/key_frame_publisher.h"
#include "publisher/key_frames_publisher.h"

#include "mapping/back_end/back_end.h"

namespace lidar_slam {
class BackEndFlow {
 public:
  explicit BackEndFlow(ros::NodeHandle& nh);
  bool Run();
  bool ForceOptimize();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateBackEnd();
  bool SaveTrajectory();
  bool PublishData();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;

  std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
  std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
  std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
  std::shared_ptr<BackEnd> back_end_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> gnss_pose_data_buff_;
  std::deque<PoseData> laser_odom_data_buff_;

  PoseData current_gnss_pose_data_;
  PoseData current_laser_odom_data_;
  CloudData current_cloud_data_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_FLOW_H_
