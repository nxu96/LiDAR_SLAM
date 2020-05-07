/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 18:59:31
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 19:23:19
 * @Description: Description
 */

#ifndef LIDAR_SLAM_INCLUDE_PUBLISHER_ODOMETRY_PUBLISHER_H_
#define LIDAR_SLAM_INCLUDE_PUBLISHER_ODOMETRY_PUBLISHER_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <string>

namespace lidar_slam {
class OdometryPublisher {
 public:
  OdometryPublisher(const ros::NodeHandle& nh, std::string topic_name,
      std::string base_frame_id, std::string child_frame_id, size_t buff_size);

  OdometryPublisher() = default;

  void Publish(const Eigen::Matrix4f& transform_matrix);

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  nav_msgs::Odometry odometry_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_PUBLISHER_ODOMETRY_PUBLISHER_H_
