/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-07 16:40:03
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 10:57:23
 * @Description: Description
 */
#include <ros/ros.h>
#include <pcl/common/transforms.h>

#include <glog/logging.h>
#include "global_definition/global_definition.h"

// pub
#include "publisher/cloud_publisher.h"
#include "publisher/odometry_publisher.h"
// sub
#include "subscriber/cloud_subscriber.h"
#include "subscriber/gnss_subscriber.h"
#include "subscriber/imu_subscriber.h"
// trans
#include "tf_listener/tf_listener.h"


using namespace lidar_slam;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = lidar_slam::WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "test_frame_node");
  ros::NodeHandle nh;

  std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
      std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr =
      std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  std::shared_ptr<IMUSubscriber> imu_sub_ptr =
      std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
  std::shared_ptr<TFListener> lidar_to_imu_ptr =
      std::make_shared<TFListener>(nh, "imu_link", "velo_link");

  std::shared_ptr<CloudPublisher> cloud_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");

  std::shared_ptr<OdometryPublisher> odom_pub_ptr =
      std::make_shared<OdometryPublisher>(nh, "lidar_odom",
      "/map", "/lidar", 100);

  std::deque<CloudData> cloud_data_buff;
  std::deque<IMUData> imu_data_buff;
  std::deque<GNSSData> gnss_data_buff;

  Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();

  bool transform_received = false;
  bool gnss_origin_position_inited = false;

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
      while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0  &&
             gnss_data_buff.size() > 0) {
        CloudData cloud = cloud_data_buff.front();
        IMUData imu = imu_data_buff.front();
        GNSSData gnss = gnss_data_buff.front();

        double time_diff = cloud.time_ - imu.time_;
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

          Eigen::Matrix4f odometry_matrix;

          if (!gnss_origin_position_inited) {
            gnss.InitOriginPosition();
            gnss_origin_position_inited = true;
          }
          gnss.UpdateXYZ();
          odometry_matrix(0, 3) = gnss.local_E_;
          odometry_matrix(1, 3) = gnss.local_N_;
          odometry_matrix(2, 3) = gnss.local_U_;
          odometry_matrix.block<3, 3>(0, 0) = imu.GetOrientationMatrix();
          odometry_matrix *= lidar_to_imu;
          // transform the point cloud from lidar frame to map frame
          pcl::transformPointCloud(*cloud.cloud_ptr_, *cloud.cloud_ptr_,
              odometry_matrix);

          // publish
          cloud_pub_ptr->Publish(cloud.cloud_ptr_);
          odom_pub_ptr->Publish(odometry_matrix);
        }
      }
    }
  }

  return 0;
}
