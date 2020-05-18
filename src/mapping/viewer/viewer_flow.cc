/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-18 10:16:25
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 11:48:48
 * @Description: Viewer Flow Implementation
 */
#include "mapping/viewer/viewer_flow.h"
#include <glog/logging.h>
#include "global_definition/global_definition.h"

namespace lidar_slam {
ViewerFlow::ViewerFlow(ros::NodeHandle& nh) {
  // subscriber
  cloud_sub_ptr_ =
    std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
  key_frame_sub_ptr_ =
    std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000);
  transformed_odom_sub_ptr_ =
    std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 100000);
  optimized_key_frames_sub_ptr_ =
    std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames",
                                            100000);
  // publisher
  optimized_odom_pub_ptr_ =
    std::make_shared<OdometryPublisher>(nh, "/optimized_odom",
                                          "/map", "/lidar", 100);
  current_scan_pub_ptr_ =
    std::make_shared<CloudPublisher>(nh, "/current_scan", 100, "/map");
  global_map_pub_ptr_ =
    std::make_shared<CloudPublisher>(nh, "/global_map", 100, "/map");
  local_map_pub_ptr_ =
    std::make_shared<CloudPublisher>(nh, "/local_map", 100, "/map");
  // viewer
  viewer_ptr_ = std::make_shared<Viewer>();
}

bool ViewerFlow::Run() {
  if (!ReadData())
    return false;

  while (HasData()) {
    if (!ValidData())
      continue;

    if (UpdateViewer()) {
      PublishData();
    }
  }

  return true;
}

bool ViewerFlow::ReadData() {
  cloud_sub_ptr_->ParseData(&cloud_data_buff_);
  transformed_odom_sub_ptr_->ParseData(transformed_odom_buff_);
  key_frame_sub_ptr_->ParseData(key_frame_buff_);
  optimized_key_frames_sub_ptr_->ParseData(optimized_key_frames_);

  return true;
}

bool ViewerFlow::HasData() {
  if (cloud_data_buff_.size() == 0)
    return false;
  if (transformed_odom_buff_.size() == 0)
    return false;

  return true;
}

bool ViewerFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_transformed_odom_ = transformed_odom_buff_.front();

  double diff_odom_time = current_cloud_data_.time_ -
                          current_transformed_odom_.time;

  if (diff_odom_time < -0.05) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_odom_time > 0.05) {
    transformed_odom_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  transformed_odom_buff_.pop_front();

  return true;
}

bool ViewerFlow::UpdateViewer() {
  return viewer_ptr_->Update(key_frame_buff_, optimized_key_frames_,
                              current_transformed_odom_,
                              current_cloud_data_);
}

bool ViewerFlow::PublishData() {
  optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose());
  current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());

  if (viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    viewer_ptr_->GetLocalMap(cloud_ptr);
    local_map_pub_ptr_->Publish(cloud_ptr);
  }

  if (viewer_ptr_->HasNewGlobalMap() &&
    global_map_pub_ptr_->HasSubscribers()) {
    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    viewer_ptr_->GetGlobalMap(cloud_ptr);
    global_map_pub_ptr_->Publish(cloud_ptr);
  }

  return true;
}

bool ViewerFlow::SaveMap() {
  return viewer_ptr_->SaveMap();
}
}  // namespace lidar_slam
