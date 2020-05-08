/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 15:54:43
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 17:35:46
 * @Description: Front end data flow management implementation
 */
#include "front_end/front_end_flow.h"
#include "glog/logging.h"

namespace lidar_slam {

FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
  // Cloud
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
  // GNSS
  gnss_sub_ptr_ =
      std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  // IMU
  imu_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
  // TF listener
  lidar_to_imu_ptr_ =
      std::make_shared<TFListener>(nh, "velo_link", "imu_link");
  // current scan
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
  // local map
  local_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "local_map", 100, "map");
  // global map
  global_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "global_map", 100, "map");
  // lidar odom
  lidar_odom_pub_ptr_ =
      std::make_shared<OdometryPublisher>(
        nh, "lidar_odom", "map", "lidar", 100);
  // gnss odom
  gnss_odom_pub_ptr_ =
      std::make_shared<OdometryPublisher>(nh, "gnss_odom", "map", "lidar", 100);
  front_end_ptr_ = std::make_shared<FrontEnd>();
  local_map_ptr_.reset(new CloudData::CLOUD());
  global_map_ptr_.reset(new CloudData::CLOUD());
  current_scan_ptr_.reset(new CloudData::CLOUD());
}

bool FrontEndFlow::Run() {
  ReadData();

  if (!Initcalibration()) {
    return false;
  }

  if (!InitGNSS()) {
    return false;
  }

  while (hasData()) {
    if (!ValidData()) {
      continue;
    }
    UpdateGNSSOdometry();
    if (UpdateLidarOdometry()) {
      PublishData();
    }
  }

  return true;
}
bool FrontEndFlow::SaveMap() {
  return front_end_ptr_->SaveMap();
}
bool FrontEndFlow::PublishGlobalMap() {
  if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) {
    global_map_pub_ptr_->Publish(global_map_ptr_);
    global_map_ptr_.reset(new CloudData::CLOUD());
  }
  return true;
}
bool FrontEndFlow::ReadData() {
  cloud_sub_ptr_->ParseData(&cloud_data_buff_);
  imu_sub_ptr_->ParseData(&imu_data_buff_);
  gnss_sub_ptr_->ParseData(&gnss_data_buff_);

  return true;
}

bool FrontEndFlow::Initcalibration() {
  static bool calibration_received = false;
  if (!calibration_received) {
    if (lidar_to_imu_ptr_->LookUpData(lidar_to_imu_)) {
      calibration_received = true;
    }
  }
  return calibration_received;
}

bool FrontEndFlow::InitGNSS() {
  static bool gnss_inited = false;
  if (!gnss_inited && !gnss_data_buff_.empty()) {
    GNSSData gnss_data = gnss_data_buff_.front();
    gnss_data.InitOriginPosition();
    gnss_inited = true;
  }
  return gnss_inited;
}

bool FrontEndFlow::hasData() {
  if (cloud_data_buff_.empty() || imu_data_buff_.empty()
    || gnss_data_buff_.empty()) {
    return false;
  }
  return true;
}

bool FrontEndFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();
  double time_diff = current_cloud_data_.time_ - current_imu_data_.time_;
  if (time_diff < -0.05) {
    // cloud data is too old throw it away
    cloud_data_buff_.pop_front();
    return false;
  } else if (time_diff > 0.05) {
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    return false;
  } else {
    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    return true;
  }
}

bool FrontEndFlow::UpdateGNSSOdometry() {
  gnss_odometry_ = Eigen::Matrix4f::Identity();
  current_gnss_data_.UpdateXYZ();
  gnss_odometry_(0, 3) = current_gnss_data_.local_E_;
  gnss_odometry_(1, 3) = current_gnss_data_.local_N_;
  gnss_odometry_(2, 3) = current_gnss_data_.local_U_;
  gnss_odometry_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
  gnss_odometry_ *= lidar_to_imu_;

  return true;
}

bool FrontEndFlow::UpdateLidarOdometry() {
  static bool front_end_pose_inited = false;
  if (!front_end_pose_inited) {
    front_end_pose_inited = true;
    front_end_ptr_->SetInitPose(gnss_odometry_);
    lidar_odometry_ = gnss_odometry_;
    return true;
  }
  lidar_odometry_ = Eigen::Matrix4f::Identity();
  if (front_end_ptr_->Update(current_cloud_data_, lidar_odometry_)) {
    return true;
  } else {
    return false;
  }
}

bool FrontEndFlow::PublishData() {
  gnss_odom_pub_ptr_->Publish(gnss_odometry_);
  lidar_odom_pub_ptr_->Publish(lidar_odometry_);

  front_end_ptr_->GetCurrentScan(current_scan_ptr_);
  cloud_pub_ptr_->Publish(current_scan_ptr_);

  if (front_end_ptr_->GetNewLocalMap(local_map_ptr_)) {
    local_map_pub_ptr_->Publish(local_map_ptr_);
  }
  return true;
}
}  // namespace lidar_slam
