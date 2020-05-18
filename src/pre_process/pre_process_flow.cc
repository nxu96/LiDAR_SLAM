/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 20:28:42
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 11:53:07
 * @Description: Pre-process data flow implementation
 */
#include "pre_process/pre_process_flow.h"
#include <glog/logging.h>
#include "global_definition/global_definition.h"

namespace lidar_slam {
PreProcessFlow::PreProcessFlow(ros::NodeHandle& nh) {
  // Cloud
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
  // IMU
  imu_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
  // Velocity
  velocity_sub_ptr_ =
      std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
  // GNSS
  gnss_sub_ptr_ =
      std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  // TF listener
  lidar_to_imu_ptr_ =
      std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");
  // publisher
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/synced_cloud", 100, "/velo_link");
  gnss_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss",
                                                     "/map", "/velo_link", 100);
  distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool PreProcessFlow::Run() {
  if (!ReadData()) {
    return false;
  }

  if (!Initcalibration()) {
    return false;
  }

  if (!InitGNSS()) {
    return false;
  }

  while (HasData()) {
    if (!ValidData()) {
      continue;
    }
    TransFormData();
    PublishData();
  }

  return true;
}

bool PreProcessFlow::ReadData() {
  cloud_sub_ptr_->ParseData(&cloud_data_buff_);
  static std::deque<IMUData> unsynced_imu_;
  static std::deque<VelocityData> unsynced_velocity_;
  static std::deque<GNSSData> unsynced_gnss_;

  imu_sub_ptr_->ParseData(&unsynced_imu_);
  gnss_sub_ptr_->ParseData(&unsynced_gnss_);
  velocity_sub_ptr_->ParseData(&unsynced_velocity_);

  if (cloud_data_buff_.empty()) {
    return false;
  }
  double sync_time = cloud_data_buff_.front().time_;
  bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, sync_time);
  bool valid_gnss = GNSSData::SyncData(
    unsynced_gnss_, gnss_data_buff_, sync_time);
  bool valid_velocity = VelocityData::SyncData(
    unsynced_velocity_, velocity_data_buff_, sync_time);
  // NOTE: The following several lines does not make sense to me
  static bool sensor_inited = false;
  if (!sensor_inited) {
    if (!valid_imu || !valid_gnss || !valid_velocity) {
      cloud_data_buff_.pop_front();
      return false;
    }
    sensor_inited = true;
  }
  return true;
}


bool PreProcessFlow::Initcalibration() {
  static bool calibration_received = false;
  if (!calibration_received) {
    if (lidar_to_imu_ptr_->LookUpData(lidar_to_imu_)) {
      calibration_received = true;
    }
  }
  return calibration_received;
}

bool PreProcessFlow::InitGNSS() {
  static bool gnss_inited = false;
  if (!gnss_inited && !gnss_data_buff_.empty()) {
    GNSSData gnss_data = gnss_data_buff_.front();
    gnss_data.InitOriginPosition();
    gnss_inited = true;
  }
  return gnss_inited;
}

bool PreProcessFlow::HasData() {
  // if any one of these buffers is empty, return false
  if (cloud_data_buff_.empty() || imu_data_buff_.empty()
    || gnss_data_buff_.empty() || velocity_data_buff_.empty()) {
    return false;
  }
  return true;
}

bool PreProcessFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();
  current_velocity_data_ = velocity_data_buff_.front();

  double diff_imu_time = current_cloud_data_.time_ - current_imu_data_.time_;
  double diff_velo_time = current_cloud_data_.time_ -
                              current_velocity_data_.time_;
  double diff_gnss_time = current_cloud_data_.time_ - current_gnss_data_.time_;

  if (diff_imu_time < -0.05 || diff_gnss_time < -0.05
      || diff_velo_time < -0.05) {
    // cloud data is too old throw it away
    cloud_data_buff_.pop_front();
    return false;
  }
  if (diff_imu_time > 0.05) {
    imu_data_buff_.pop_front();
    return false;
  }
  if (diff_gnss_time > 0.05) {
    gnss_data_buff_.pop_front();
    return false;
  }
  if (diff_velo_time > 0.05) {
    velocity_data_buff_.pop_front();
    return false;
  }
  // NOTE: All checks passed!
  // then we could use all these four data to perform the following operations
  // since they have been saved in the class as current_xxx_data_, we should
  // remove them from the buffer.
  cloud_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  velocity_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool PreProcessFlow::TransFormData() {
  gnss_pose_ = Eigen::Matrix4f::Identity();
  current_gnss_data_.UpdateXYZ();
  gnss_pose_(0, 3) = current_gnss_data_.local_E_;
  gnss_pose_(1, 3) = current_gnss_data_.local_N_;
  gnss_pose_(2, 3) = current_gnss_data_.local_U_;
  gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
  gnss_pose_ *= lidar_to_imu_;

  // NOTE: Point Cloud Distortion adjust
  current_velocity_data_.TransformCoordinate(lidar_to_imu_.inverse());
  const float SCAN_PERIOD = 0.1;  // 100 ms period
  distortion_adjust_ptr_->SetMotionInfo(SCAN_PERIOD, current_velocity_data_);
  distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr_,
                                      current_cloud_data_.cloud_ptr_);

  return true;
}


bool PreProcessFlow::PublishData() {
  // Publish current synced point cloud for front end
  cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr_,
                          current_cloud_data_.time_);
  // Publish GNSS odometry (gnss pose)
  gnss_odom_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time_);

  return true;
}
}  // namespace lidar_slam
