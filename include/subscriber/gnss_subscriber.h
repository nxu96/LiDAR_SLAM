/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 18:26:08
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 18:30:56
 * @Description: Description
 */

#ifndef LIDAR_SLAM_INCLUDE_SUBSCRIBER_GNSS_SUBSCIBER_H_
#define LIDAR_SLAM_INCLUDE_SUBSCRIBER_GNSS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <deque>
#include "sensor_data/gnss_data.h"

namespace lidar_slam {
class GNSSSubscriber {
 public:
  GNSSSubscriber(const ros::NodeHandle& nh, std::string topic_name,
                  size_t buff_size);
  GNSSSubscriber() = default;
  void ParseData(std::deque<GNSSData>* deque_gnss_data);

 private:
  void OnGNSS(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

 private:
  ros::Subscriber subscriber_;
  ros::NodeHandle nh_;
  std::deque<GNSSData> new_gnss_data_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SUBSCRIBER_GNSS_SUBSCRIBER_H_
