/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 00:31:20
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 12:56:39
 * @Description: Point cloud subscriber header file
 */

#ifndef LIDAR_SLAM_INCLUDE_SUBSCRIBER_CLOUD_SUBSCRIBER_H_
#define LIDAR_SLAM_INCLUDE_SUBSCRIBER_CLOUD_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <deque>

#include "sensor_data/cloud_data.h"

namespace lidar_slam {
class CloudSubscriber {
 public:
  CloudSubscriber(const ros::NodeHandle& nh, std::string topic,
                  size_t buff_size);
  CloudSubscriber() = default;
  void ParseData(std::deque<CloudData>* deque_cloud_data);

 private:
  void OnPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::deque<CloudData> new_cloud_data_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SUBSCRIBER_CLOUD_SUBSCRIBER_H_
