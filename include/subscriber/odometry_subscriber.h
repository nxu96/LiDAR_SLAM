/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 15:38:18
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 11:40:01
 * @Description: Subscribe to the odometry data
 */
#ifndef LIDAR_SLAM_INCLUDE_SUBSCRIBER_ODOMETRY_SUBSCRIBER_H_
#define LIDAR_SLAM_INCLUDE_SUBSCRIBER_ODOMETRY_SUBSCRIBER_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <string>
#include "sensor_data/pose_data.h"

namespace lidar_slam {
class OdometrySubscriber {
 public:
  OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name,
                     std::size_t buff_size);
  OdometrySubscriber() = default;
  void ParseData(std::deque<PoseData>& deque_pose_data);

 private:
  void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::deque<PoseData> new_pose_data_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SUBSCRIBER_ODOMETRY_SUBSCRIBER_H_
