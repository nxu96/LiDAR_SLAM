/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 22:37:28
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 22:48:58
 * @Description: Backend header file
 */
#ifndef LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_H_
#define LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_H_

#include <yaml-cpp/yaml.h>
#include <deque>
#include <string>
#include <fstream>

#include "sensor_data/cloud_data.h"
#include "sensor_data/pose_data.h"
#include "sensor_data/key_frame.h"

namespace lidar_slam {
class BackEnd {
 public:
  BackEnd();

  bool Update(const CloudData& cloud_data, const PoseData& lidar_odom,
              const PoseData& gnss_pose);

  void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);
  bool HasNewKeyFrame();
  bool HasNewOptimized();
  void GetLatestKeyFrame(KeyFrame& key_frame);

 private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node& config_node);
  bool InitDataPath(const YAML::Node& config_node);

  void ResetParam();
  bool SaveTrajectory(const PoseData& lidar_odom, const PoseData& gnss_pose);
  bool MaybeNewKeyFrame(const CloudData& cloud_data,
                        const PoseData& lidar_odom);
  bool MaybeOptimized();

 private:
  std::string key_frames_path_ = "";
  std::string trajectory_path_ = "";

  std::ofstream ground_truth_ofs_;
  std::ofstream laser_odom_ofs_;

  float key_frame_distance_ = 2.0;
  int optimize_step_with_none_ = 100;
  int optimize_step_with_gnss_ = 100;
  int optimize_step_with_loop_ = 10;

  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;

  Eigen::Matrix4f last_key_pose_ = Eigen::Matrix4f::Identity();
  KeyFrame latest_key_frame_;
  std::deque<KeyFrame> key_frames_deque_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_H_
