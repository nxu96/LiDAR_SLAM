/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 22:37:28
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-19 12:57:59
 * @Description: Backend header file
 */
#ifndef LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_H_
#define LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_H_

#include <yaml-cpp/yaml.h>
#include <deque>
#include <string>
#include <fstream>
#include <memory>

#include "sensor_data/cloud_data.h"
#include "sensor_data/pose_data.h"
#include "sensor_data/key_frame.h"
#include "models/graph_optimizer/g2o/g2o_graph_optimizer.h"


namespace lidar_slam {
class BackEnd {
 public:
  BackEnd();

  bool Update(const CloudData& cloud_data, const PoseData& lidar_odom,
              const PoseData& gnss_pose);
  bool ForceOptimize();
  void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);
  bool HasNewKeyFrame();
  bool HasNewOptimized();
  void GetLatestKeyFrame(KeyFrame& key_frame);

 private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node& config_node);
  bool InitGraphOptimizer(const YAML::Node& config_node);
  bool InitDataPath(const YAML::Node& config_node);

  void ResetParam();
  bool SaveTrajectory(const PoseData& lidar_odom, const PoseData& gnss_pose);
  bool AddNodeAndEdge(const PoseData& gnss_data);
  bool MaybeNewKeyFrame(const CloudData& cloud_data,
                        const PoseData& lidar_odom);
  bool MaybeOptimized();

 private:
  std::string key_frames_path_ = "";
  std::string trajectory_path_ = "";

  std::ofstream ground_truth_ofs_;
  std::ofstream laser_odom_ofs_;

  float key_frame_distance_ = 2.0;

  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;

  KeyFrame current_key_frame_;

  // Eigen::Matrix4f last_key_pose_ = Eigen::Matrix4f::Identity();
  // KeyFrame latest_key_frame_;
  std::deque<KeyFrame> key_frames_deque_;
  // General Purpose Optimizer
  std::shared_ptr<InterfaceGraphOptimizer> graph_optimizer_ptr_;
  // NOTE: struct-like class for all config params
  // what is the benefit of doing this btw?
  class GraphOptimizerConfig {
   public:
    GraphOptimizerConfig() {
      odom_edge_noise.resize(6);
      close_loop_noise.resize(6);
      gnss_noise.resize(3);
    }

   public:
    bool use_gnss = true;
    bool use_loop_close = false;

    Eigen::VectorXd odom_edge_noise;
    Eigen::VectorXd close_loop_noise;
    Eigen::VectorXd gnss_noise;

    int optimize_step_with_key_frame = 100;
    int optimize_step_with_gnss = 100;
    int optimize_step_with_loop = 10;
  };
  GraphOptimizerConfig graph_optimizer_config_;

  int new_gnss_cnt_ = 0;
  int new_loop_cnt_ = 0;
  int new_key_frame_cnt_ = 0;
};

}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_MAPPING_BACK_END_BACK_END_H_
