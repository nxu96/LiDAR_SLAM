/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 18:59:31
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 14:52:32
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
                    std::string base_frame_id, std::string child_frame_id,
                    size_t buff_size);

  OdometryPublisher() = default;

  void Publish(const Eigen::Matrix4f& transform_matrix);
  void Publish(const Eigen::Matrix4f& transform_matrix, double time);
  bool HasSubscribers();

 private:
  void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  nav_msgs::Odometry odometry_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_PUBLISHER_ODOMETRY_PUBLISHER_H_
