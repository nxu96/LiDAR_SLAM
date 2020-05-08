/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-07 19:44:44
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 00:05:17
 * @Description: Front End Node
 */

#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "global_definition/global_definition.h.in"
// pub
#include "publisher/cloud_publisher.h"
#include "publisher/odometry_publisher.h"
// sub
#include "subscriber/cloud_subscriber.h"
#include "subscriber/gnss_subscriber.h"
#include "subscriber/imu_subscriber.h"
// trans
#include "tf_listener/tf_listener.h"
#include "front_end/front_end.h"

using namespace lidar_slam;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = lidar_slam::WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "front_end_node");
  ros::NodeHandle nh;
  // Cloud
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
      std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
  // GNSS
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr =
      std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  // IMU
  std::shared_ptr<IMUSubscriber> imu_sub_ptr =
      std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
  // TF listener
  std::shared_ptr<TFListener> lidar_to_imu_ptr =
      std::make_shared<TFListener>(nh, "velo_link", "imu_link");
  // current scan
  std::shared_ptr<CloudPublisher> cloud_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
  // local map
  std::shared_ptr<CloudPublisher> local_map_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "local_map", 100, "map");
  // global map
  std::shared_ptr<CloudPublisher> global_map_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "global_map", 100, "map");
  // lidar odom
  std::shared_ptr<OdometryPublisher> lidar_odom_pub_ptr =
      std::make_shared<OdometryPublisher>(
        nh, "lidar_odom", "map", "lidar", 100);
  // gnss odom
  std::shared_ptr<OdometryPublisher> gnss_odom_pub_ptr =
      std::make_shared<OdometryPublisher>(nh, "gnss_odom", "map", "lidar", 100);
  
  std::shared_ptr<FrontEnd> front_end_ptr = std::make_shared<FrontEnd>();

  std::deque<CloudData> cloud_data_buff;
  std::deque<IMUData> imu_data_buff;
  std::deque<GNSSData> gnss_data_buff;

  Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
  bool transform_received = false;
  bool gnss_origin_position_inited = false;
  bool front_end_pose_inited = false;
  CloudData::CLOUD_PTR local_map_ptr(new CloudData::CLOUD());
  CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
  CloudData::CLOUD_PTR current_scan_ptr(new CloudData::CLOUD());
  double run_time = 0.0;
  double init_time = 0.0;
  bool time_inited = false;
  bool has_global_map_published = false;

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    cloud_sub_ptr->ParseData(&cloud_data_buff);
    imu_sub_ptr->ParseData(&imu_data_buff);
    gnss_sub_ptr->ParseData(&gnss_data_buff);

    if (!transform_received) {
      if (lidar_to_imu_ptr->LookUpData(lidar_to_imu)) {
        transform_received = true;
      }
    } else {
      while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0
        && gnss_data_buff.size() > 0) {
        CloudData cloud_data = cloud_data_buff.front();
        IMUData imu_data = imu_data_buff.front();
        GNSSData gnss_data = gnss_data_buff.front();
        if (!time_inited) {
          time_inited = true;
          init_time = cloud_data.time_;
        } else {
          run_time = cloud_data.time_ - init_time;
        }

        double time_diff = cloud_data.time_ - imu_data.time_;
         if (time_diff < -0.05) {
          // cloud data is too old throw it away
          cloud_data_buff.pop_front();
        } else if (time_diff > 0.05) {
          imu_data_buff.pop_front();
          gnss_data_buff.pop_front();
        } else {
          cloud_data_buff.pop_front();
          imu_data_buff.pop_front();
          gnss_data_buff.pop_front();

          Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();
          if (!gnss_origin_position_inited) {
            gnss_data.InitOriginPosition();
            gnss_origin_position_inited = true;
          }
          gnss_data.UpdateXYZ();
          odom(0, 3) = gnss_data.local_E_;
          odom(1, 3) = gnss_data.local_N_;
          odom(2, 3) = gnss_data.local_U_;
          odom.block<3, 3>(0, 0) = imu_data.GetOrientationMatrix();
          odom *= lidar_to_imu;
          gnss_odom_pub_ptr->Publish(odom);
          // NOTE: Front End Matching
          if (!front_end_pose_inited) {
            front_end_ptr->SetInitPose(odom);
            front_end_pose_inited = true;
          }
          front_end_ptr->SetPredictPose(odom);
          Eigen::Matrix4f lidar_odom =
            front_end_ptr->Update(cloud_data);
          lidar_odom_pub_ptr->Publish(lidar_odom);

          front_end_ptr->GetCurrentScan(current_scan_ptr);
          cloud_pub_ptr->Publish(current_scan_ptr);

          if (front_end_ptr->GetNewLocalMap(local_map_ptr)) {
            local_map_pub_ptr->Publish(local_map_ptr);
          }
        }
        std::cout << "[Run Time] " << run_time << std::endl;
        if (run_time > 460.0 && !has_global_map_published) {
          if (front_end_ptr->GetCurrentScan(global_map_ptr)) {
            has_global_map_published = true;
            global_map_pub_ptr->Publish(global_map_ptr);
          }
        }
      }
    }
    rate.sleep();
  }
  return 0;
}
