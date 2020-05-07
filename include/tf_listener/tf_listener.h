/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 19:29:53
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 19:36:49
 * @Description: Description
 */

#ifndef LIDAR_SLAM_INCLUDE_TF_LISTENER_TF_LISTENER_H_
#define LIDAR_SLAM_INCLUDE_TF_LISTENER_TF_LISTENER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <string>

namespace lidar_slam {
class TFListener {
 public:
  TFListener(const ros::NodeHandle& nh, std::string base_frame_id,
      std::string child_frame_id);

  TFListener() = default;

  bool LookUpData(Eigen::Matrix4f& transform_matrix);

 private:
  bool TransformToMatrix(const tf::StampedTransform& transform,
      Eigen::Matrix4f& transform_matrix);
 private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string base_frame_id_;
  std::string child_frame_id_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_TF_LISTENER_TF_LISTENER_H_
