/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 23:06:33
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-19 11:29:59
 * @Description: Back End Flow Implementation
 */
#include "mapping/back_end/back_end_flow.h"
#include <glog/logging.h>
#include "tools/file_manager.h"
#include "global_definition/global_definition.h"

namespace lidar_slam {
BackEndFlow::BackEndFlow(ros::NodeHandle& nh) {
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
  // use gnss pose as constraints in the graph optimizer
  gnss_pose_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
  // input lidar odometry
  laser_odom_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/laser_odom", 100000);
  // updated odometry pose?
  transformed_odom_pub_ptr_ =
      std::make_shared<OdometryPublisher>(nh, "/transformed_odom",
                                          "/map", "/lidar", 100);
  key_frame_pub_ptr_ =
      std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
  key_frames_pub_ptr_ =
      std::make_shared<KeyFramesPublisher>(nh, "/optimized_key_frames",
                                            "/map", 100);
  back_end_ptr_ = std::make_shared<BackEnd>();
}

bool BackEndFlow::Run() {
  if (!ReadData())
    return false;

  while (HasData()) {
    if (!ValidData()) {
      continue;
    }

    UpdateBackEnd();

    PublishData();
  }

  return true;
}

bool BackEndFlow::ForceOptimize() {
  back_end_ptr_->ForceOptimize();
  if (back_end_ptr_->HasNewOptimized()) {
    std::deque<KeyFrame> optimized_key_frames;
    back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
    key_frames_pub_ptr_->Publish(optimized_key_frames);
  }
  return true;
}

bool BackEndFlow::ReadData() {
  cloud_sub_ptr_->ParseData(&cloud_data_buff_);
  gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
  laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);

  return true;
}

bool BackEndFlow::HasData() {
  if (cloud_data_buff_.size() == 0)
    return false;
  if (gnss_pose_data_buff_.size() == 0)
    return false;
  if (laser_odom_data_buff_.size() == 0)
    return false;

  return true;
}

bool BackEndFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_gnss_pose_data_ = gnss_pose_data_buff_.front();
  current_laser_odom_data_ = laser_odom_data_buff_.front();

  double diff_gnss_time = current_cloud_data_.time_ -
                          current_gnss_pose_data_.time;
  double diff_laser_time = current_cloud_data_.time_ -
                           current_laser_odom_data_.time;

  if (diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
      cloud_data_buff_.pop_front();
      return false;
  }

  if (diff_gnss_time > 0.05) {
      gnss_pose_data_buff_.pop_front();
      return false;
  }

  if (diff_laser_time > 0.05) {
      laser_odom_data_buff_.pop_front();
      return false;
  }

  cloud_data_buff_.pop_front();
  gnss_pose_data_buff_.pop_front();
  laser_odom_data_buff_.pop_front();

  return true;
}

bool BackEndFlow::UpdateBackEnd() {
  static bool odometry_inited = false;
  static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

  if (!odometry_inited) {
      odometry_inited = true;
      odom_init_pose = current_gnss_pose_data_.pose *
                       current_laser_odom_data_.pose.inverse();
  }
  current_laser_odom_data_.pose = odom_init_pose *
                                  current_laser_odom_data_.pose;

  return back_end_ptr_->Update(current_cloud_data_, current_laser_odom_data_,
                               current_gnss_pose_data_);
}

bool BackEndFlow::PublishData() {
  transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose,
                                     current_laser_odom_data_.time);

  if (back_end_ptr_->HasNewKeyFrame()) {
      KeyFrame key_frame;
      back_end_ptr_->GetLatestKeyFrame(key_frame);
      key_frame_pub_ptr_->Publish(key_frame);
  }

  if (back_end_ptr_->HasNewOptimized()) {
      std::deque<KeyFrame> optimized_key_frames;
      back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
      key_frames_pub_ptr_->Publish(optimized_key_frames);
  }

  return true;
}
}  // namespace lidar_slam
