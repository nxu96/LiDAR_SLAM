/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-09 10:14:27
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-09 13:39:40
 * @Description: Subscribe to the velocity data?
 */
#ifndef LIDAR_SLAM_INCLUDE_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_
#define LIDAR_SLAM_INCLUDE_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_

#include <ros/ros.h>
#include <deque>
#include <string>
#include "geometry_msgs/TwistStamped.h"
#include "sensor_data/velocity_data.h"

namespace lidar_slam {
class VelocitySubscriber {
 public:
  VelocitySubscriber(const ros::NodeHandle& nh, std::string topic_name,
    std::size_t buff_size);
  VelocitySubscriber() = default;
  void ParseData(std::deque<VelocityData>* deque_velocity_data);

 private:
  void OnVelocity(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::deque<VelocityData> new_velocity_data_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_
