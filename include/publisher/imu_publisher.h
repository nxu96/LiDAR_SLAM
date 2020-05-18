/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 13:59:59
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 14:52:56
 * @Description: IMU publisher header file
 */
#ifndef LIDAR_SLAM_INCLUDE_PUBLISHER_IMU_PUBLISHER_H_
#define LIDAR_SLAM_INCLUDE_PUBLISHER_IMU_PUBLISHER_H_
#include <ros/ros.h>
// ROS imu msg
#include <sensor_msgs/Imu.h>
#include <string>
// my IMU data class
#include "sensor_data/imu_data.h"

namespace lidar_slam {
class IMUPublisher {
 public:
  IMUPublisher(ros::NodeHandle& nh, std::string topic_name,
               std::size_t buff_size, std::string frame_id);
  IMUPublisher() = default;
  void Publish(IMUData imu_data);
  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string frame_id_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_PUBLISHER_IMU_PUBLISHER_H_
