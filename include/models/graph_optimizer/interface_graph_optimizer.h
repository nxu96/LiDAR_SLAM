/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-18 21:35:31
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 21:52:28
 * @Description: Graph Optimizer Interface Header file
 */
#ifndef LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_H_
#define LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_H_

#include <Eigen/Dense>
#include <string>
#include <deque>

namespace lidar_slam {
class InterfaceGraphOptimizer {
 public:
  // Set destructor of base class as virtual function!
  virtual ~InterfaceGraphOptimizer() {}
  // optimization main function
  virtual bool Optimize() = 0;
  // data I/O pure virtual function
  virtual bool GetOptimizedPose(
      std::deque<Eigen::Matrix4f>& optimized_pose) = 0;
  virtual int GetNodeNum() = 0;
  // Add node, edge and robust kernel
  virtual void SetEdgeRobustKernel(std::string robust_kernel_name,
                                   double robust_kernel_size) = 0;
  // NOTE: Isometry3d is the 4X4 transform matrix
  virtual void AddSE3Node(const Eigen::Isometry3d &pose, bool need_fix) = 0;
  virtual void AddSE3Edge(int vertex_index1,
                          int vertex_index2,
                          const Eigen::Isometry3d &relative_pose,
                          const Eigen::VectorXd noise) = 0;
  // NOTE: prior means we are using the measurement, so this is the edge
  // connecting odometry pose and measurement pose I guess
  virtual void AddSE3PriorXYZEdge(int se3_vertex_index,
                                  const Eigen::Vector3d &xyz,
                                  Eigen::VectorXd noise) = 0;
  virtual void AddSE3PriorQuaternionEdge(int se3_vertex_index,
                                          const Eigen::Quaterniond &quat,
                                          Eigen::VectorXd noise) = 0;
  // Set optimization parameter
  void SetMaxIterationsNum(int max_iterations_num);

  // Protected member. Derived class can access this member
 protected:
  int max_iterations_num_ = 512;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_H_
