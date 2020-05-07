/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 19:37:28
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 19:52:21
 * @Description: Description
 */

#include "tf_listener/tf_listener.h"

namespace lidar_slam {
TFListener::TFListener(const ros::NodeHandle& nh, std::string base_frame_id,
    std::string child_frame_id) :
    nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
}

bool TFListener::LookUpData(Eigen::Matrix4f& transform_matrix) {
  // try and catch exception
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform(base_frame_id_, child_frame_id_,
                              ros::Time(0), transform);
    TransformToMatrix(transform, transform_matrix);
    return true;
  } catch (tf::TransformException &ex) {
    return false;
  }
}

bool TFListener::TransformToMatrix(const tf::StampedTransform& transform,
    Eigen::Matrix4f& transform_matrix) {
  Eigen::Translation3f trans(transform.getOrigin().getX(),
    transform.getOrigin().getY(), transform.getOrigin().getZ());
  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());

  // this matrix is the transform from child frame to the base frame
  transform_matrix = (trans * rot_z * rot_y * rot_x).matrix();
  return true;
}

}  // namespace lidar_slam
