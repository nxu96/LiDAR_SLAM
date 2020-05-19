/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 13:38:50
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 21:54:36
 * @Description: Viewer flow heaer file
 */
#ifndef LIDAR_SLAM_INCLUDE_MAPPING_VIEWER_VIEWER_FLOW_H_
#define LIDAR_SLAM_INCLUDE_MAPPING_VIEWER_VIEWER_FLOW_H_
#include <ros/ros.h>
#include <deque>
#include <memory>
// subscriber
#include "subscriber/cloud_subscriber.h"
#include "subscriber/odometry_subscriber.h"
#include "subscriber/key_frame_subscriber.h"
#include "subscriber/key_frames_subscriber.h"
// publisher
#include "publisher/odometry_publisher.h"
#include "publisher/cloud_publisher.h"
// viewer
#include "mapping/viewer/viewer.h"

namespace lidar_slam {
class ViewerFlow {
 public:
  explicit ViewerFlow(ros::NodeHandle& nh);
  bool Run();
  bool SaveMap();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool PublishGlobalData();
  bool PublishLocalData();

 private:
  // subscriber
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
  std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;
  // publisher
  std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
  std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
  // viewer
  std::shared_ptr<Viewer> viewer_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> transformed_odom_buff_;
  std::deque<KeyFrame> key_frame_buff_;
  std::deque<KeyFrame> optimized_key_frames_;
  std::deque<KeyFrame> all_key_frames_;

  CloudData current_cloud_data_;
  PoseData current_transformed_odom_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_MAPPING_VIEWER_VIEWER_FLOW_H_
