/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-14 22:41:26
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 16:14:39
 * @Description: Point Cloud Distortion Adjust Implementation
 */

#include <glog/logging.h>
#include "models/scan_adjust/distortion_adjust.h"

namespace lidar_slam {
// Set the motion info for this point cloud
void DistortionAdjust::SetMotionInfo(float scan_period,
    VelocityData velo_data) {
  scan_period_ = scan_period;
  velocity_ << velo_data.linear_velocity_.x_,
               velo_data.linear_velocity_.y_,
               velo_data.linear_velocity_.z_;
  angular_rate_ << velo_data.angular_velocity_.x_,
                   velo_data.angular_velocity_.y_,
                   velo_data.angular_velocity_.z_;
}

bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr,
    CloudData::CLOUD_PTR& output_cloud_ptr) {
  // cp the input point cloud
  CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
  // NOTE: Input and output point cloud pointer is the same thing
  // clear the output point cloud, make it ready for new points
  // output_cloud_ptr->points.clear();
  // NOTE: Here we use reset instead of clearing
  // for the sake of saving memory. Think about vector capacity and size stuff.
  output_cloud_ptr.reset(new CloudData::CLOUD());
  // input_cloud_ptr->points.clear();
  float angle_space = 2.0 * M_PI;
  // float delete_space = 5.0 * M_PI / 180.0;
  // NOTE: Start orientation
  // float start_angle = atan2(origin_cloud_ptr->points[0].y,
                            // origin_cloud_ptr->points[0].x);
  // A rotation vector
  // Eigen::AngleAxisf t_V(start_angle, Eigen::Vector3f::UnitZ());
  // Rotation matrix is from current frame to the origin frame
  // Eigen::Matrix3f rotation_matrix = t_V.matrix();
  // Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  // NOTE: the inverse because we want to rotate the lidar point
  // cloud back to origin, yes
  // transform_matrix.block<3, 3>(0, 0) = rotation_matrix.inverse();
  // pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr,
    // transform_matrix);
  // change the velocity frame a certain angle frame to zero frame
  // velocity_ = rotation_matrix * velocity_;
  // angular_rate_ = rotation_matrix * angular_rate_;

  for (std::size_t idx = 0; idx < origin_cloud_ptr->points.size(); ++idx) {
    float orient = atan2(origin_cloud_ptr->points[idx].y,
                         origin_cloud_ptr->points[idx].x);
    // -pi ~ pi to 0 ~ 2pi
    if (orient < 0.0) {
      orient += 2.0 * M_PI;
    }
    // TODO(nxu): Why do we skip these area +- 5 degrees for example
    // if (orient < delete_space || 2.0 * M_PI - orient < delete_space) {
    //   continue;
    // }
    // NOTE: Because kitti2bag use the middle point time for each lidar scan
    // So we have to subtract this time offset (scan_period / 2.0)
    float real_time = fabs(orient) / angle_space * scan_period_
                      - scan_period_ / 2.0;
    Eigen::Vector3f origin_point(origin_cloud_ptr->points[idx].x,
                                 origin_cloud_ptr->points[idx].y,
                                 origin_cloud_ptr->points[idx].z);
    Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
    Eigen::Vector3f rotated_point = current_matrix * origin_point;
    Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;
    CloudData::POINT pt;
    pt.x = adjusted_point(0);
    pt.y = adjusted_point(1);
    pt.z = adjusted_point(2);
    output_cloud_ptr->points.push_back(pt);
  }
  // pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr,
                          //  transform_matrix.inverse());
  return true;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {
  Eigen::Vector3f angle = angular_rate_ * real_time;
  Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf t_V;
  t_V = t_Vz * t_Vy * t_Vx;
  return t_V.matrix();
}
}  // namespace lidar_slam
