/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-08 15:54:43
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 11:43:28
 * @Description: Front end data flow management implementation
 */
#include "mapping/front_end/front_end_flow.h"
#include <glog/logging.h>
#include "tools/file_manager.h"
// global definition
#include "global_definition/global_definition.h"

namespace lidar_slam {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
  // Cloud
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
  // lidar odom
  lidar_odom_pub_ptr_ =
      std::make_shared<OdometryPublisher>(
        nh, "/laser_odom", "/map", "/lidar", 100);
  front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
  if (!ReadData()) {
    return false;
  }
  while (HasData()) {
    if (!ValidData()) {
      continue;
    }
    if (UpdateLidarOdometry()) {
      PublishData();
    }
  }
  return true;
}

bool FrontEndFlow::ReadData() {
  cloud_sub_ptr_->ParseData(&cloud_data_buff_);

  return true;
}

bool FrontEndFlow::HasData() {
  return cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
  // NOTE: Basicall all valid...
  current_cloud_data_ = cloud_data_buff_.front();
  cloud_data_buff_.pop_front();

  return true;
}

bool FrontEndFlow::UpdateLidarOdometry() {
  static bool front_end_pose_inited = false;
  if (!front_end_pose_inited) {
    front_end_pose_inited = true;
    front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
    return front_end_ptr_->Update(current_cloud_data_, lidar_odometry_);
  }
  return front_end_ptr_->Update(current_cloud_data_, lidar_odometry_);
}

bool FrontEndFlow::PublishData() {
  lidar_odom_pub_ptr_->Publish(lidar_odometry_, current_cloud_data_.time_);

  return true;
}

/*
bool FrontEndFlow::SaveTrajectory() {
  static std::ofstream ground_truth, lidar_odom;
  static bool is_file_created = false;
  if (!is_file_created) {
    if (!FileManager::CreateDirectory(WORK_SPACE_PATH+"/data/trajectory")) {
      return false;
    }
    if (!FileManager::CreateFile(ground_truth,
      WORK_SPACE_PATH+"/data/trajectory/ground_truth.txt")) {
      return false;
    }
    if (!FileManager::CreateFile(lidar_odom,
      WORK_SPACE_PATH+"/data/trajectory/lidar_odom.txt")) {
      return false;
    }
    is_file_created = true;
  }
  for (int i = 0; i < 3; ++i) {  // save the 3X4 matrix as a row in the txt file
    for (int j = 0; j < 4; ++j) {
      // NOTE: Use the gnss odometry as the ground truth pose;
      ground_truth << gnss_odometry_(i, j);
      lidar_odom << lidar_odometry_(i, j);
      if (i == 2 && j == 3) {
        ground_truth << std::endl;
        lidar_odom << std::endl;
      } else {
        ground_truth << " ";
        lidar_odom << " ";
      }
    }
  }

  return true;
}
*/


}  // namespace lidar_slam
