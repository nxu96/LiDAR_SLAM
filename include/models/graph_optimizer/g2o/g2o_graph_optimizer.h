/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-19 09:27:13
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-19 12:54:07
 * @Description: g2o graph optimizer header file, g2o library wrapper
 */
#ifndef LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_G2O_G2O_GRAPH_OPTIMIZER_H_
#define LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_G2O_G2O_GRAPH_OPTIMIZER_H_
// g2o stuff
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <deque>
#include <memory>
#include <string>

#include "models/graph_optimizer/g2o/edge/edge_se3_priorxyz.h"
#include "models/graph_optimizer/g2o/edge/edge_se3_priorquat.h"
#include "models/graph_optimizer/interface_graph_optimizer.h"

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
}  // namespace g2o

G2O_USE_TYPE_GROUP(slam3d);

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

// namespace g2o {
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
// } // namespace g2o

namespace lidar_slam {
class G2oGraphOptimizer: public InterfaceGraphOptimizer {
 public:
  explicit G2oGraphOptimizer(const std::string &solver_type = "lm_var");
  // Optimize main function
  bool Optimize() override;
  // Data Output
  bool GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) override;
  int GetNodeNum() override;
  // Add Edge, node, robust kernel
  void SetEdgeRobustKernel(std::string robust_kernel_name,
                           double robust_kernel_size) override;
  void AddSE3Node(const Eigen::Isometry3d &pose, bool need_fix) override;
  void AddSE3Edge(int vertex_index1,
                  int vertex_index2,
                  const Eigen::Isometry3d &relative_pose,
                  const Eigen::VectorXd noise) override;
  void AddSE3PriorXYZEdge(int se3_vertex_index,
                          const Eigen::Vector3d &xyz,
                          Eigen::VectorXd noise) override;
  void AddSE3PriorQuaternionEdge(int se3_vertex_index,
                                  const Eigen::Quaterniond &quat,
                                  Eigen::VectorXd noise) override;

 private:
  // NOTE: Pay attention to the implementation of these functions
  Eigen::MatrixXd CalculateSE3EdgeInformationMatrix(Eigen::VectorXd noise);
  Eigen::MatrixXd CalculateSE3PriorQuaternionEdgeInformationMatrix(
      Eigen::VectorXd noise);
  Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);
  void AddRobustKernel(g2o::OptimizableGraph::Edge *edge,
                       const std::string &kernel_type, double kernel_size);

 private:
  g2o::RobustKernelFactory *robust_kernel_factory_;
  std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;

  std::string robust_kernel_name_;
  double robust_kernel_size_;
  bool need_robust_kernel_ = false;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_G2O_G2O_GRAPH_OPTIMIZER_H_
