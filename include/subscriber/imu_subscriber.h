/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 13:13:12
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 18:27:44
 * @Description: IMU Subscriber class header file
 */

#ifndef LIDAR_SLAM_INCLUDE_SUBSCRIBER_IMU_SUBSCRIBER_H_
#define LIDAR_SLAM_INCLUDE_SUBSCRIBER_IMU_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <deque>
#include "sensor_data/imu_data.h"

namespace lidar_slam {
class IMUSubscriber {
 public:
  IMUSubscriber(const ros::NodeHandle& nh, std::string topic_name,
                  size_t buff_size);
  IMUSubscriber() = default;
  void ParseData(std::deque<IMUData>* deque_imu_data);

 private:
  void OnIMU(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

 private:
  ros::Subscriber subscriber_;
  ros::NodeHandle nh_;
  std::deque<IMUData> new_imu_data_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SUBSCRIBER_IMU_SUBSCRIBER_H_
