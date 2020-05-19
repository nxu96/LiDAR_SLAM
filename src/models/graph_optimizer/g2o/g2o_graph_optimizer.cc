/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-19 09:59:47
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-19 10:50:54
 * @Description: g2o graph optimizer wrapper implementation
 */

#include "models/graph_optimizer/g2o/g2o_graph_optimizer.h"
#include <glog/logging.h>
#include "tools/timer.h"

namespace lidar_slam {
G2oGraphOptimizer::G2oGraphOptimizer(const std::string &solver_type) {
  graph_ptr_.reset(new g2o::SparseOptimizer());

  g2o::OptimizationAlgorithmFactory *solver_factory =
      g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm *solver =
      solver_factory->construct(solver_type, solver_property);
  graph_ptr_->setAlgorithm(solver);

  if (!graph_ptr_->solver()) {
      LOG(ERROR) << "[g2o] Fail to create G2O optimizer！";
  }
  robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

// NOTE: Optimization main function
bool G2oGraphOptimizer::Optimize() {
    static int optimize_cnt = 0;
    // If there is no edge, no need to optimize anymore
    if (graph_ptr_->edges().size() < 1) {
        return false;
    }

    Timer optimize_time;
    graph_ptr_->initializeOptimization();
    graph_ptr_->computeInitialGuess();
    graph_ptr_->computeActiveErrors();
    graph_ptr_->setVerbose(false);

    double chi2 = graph_ptr_->chi2();
    int iterations = graph_ptr_->optimize(max_iterations_num_);

    LOG(INFO) << std::endl << "------ Finished " << ++optimize_cnt
              << "th optimization -------" << std::endl
              << "Vertex Num： " << graph_ptr_->vertices().size()
              << ", Edge Num： " << graph_ptr_->edges().size() << std::endl
              << "Iter Num： " << iterations << "/" << max_iterations_num_
              << std::endl
              << "Time Spent： " << optimize_time.Toc() << std::endl
              << "Error Diff： " << chi2 << " ---> " << graph_ptr_->chi2()
              << std::endl << std::endl;

    return true;
}

bool G2oGraphOptimizer::GetOptimizedPose(
    std::deque<Eigen::Matrix4f>& optimized_pose) {
  optimized_pose.clear();
  int vertex_num = graph_ptr_->vertices().size();
  // For each vertex
  for (int i = 0; i < vertex_num; i++) {
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(i));
    Eigen::Isometry3d pose = v->estimate();
    // isometry to eigen matrix4f pose.matrix().cast<float>()
    optimized_pose.push_back(pose.matrix().cast<float>());
  }
  return true;
}

int G2oGraphOptimizer::GetNodeNum() {
  return graph_ptr_->vertices().size();
}

void G2oGraphOptimizer::AddSE3Node(const Eigen::Isometry3d &pose,
                                   bool need_fix) {
  g2o::VertexSE3 *vertex(new g2o::VertexSE3());
  vertex->setId(graph_ptr_->vertices().size());
  vertex->setEstimate(pose);
  if (need_fix) {
      vertex->setFixed(true);
  }
  graph_ptr_->addVertex(vertex);
}

void G2oGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name,
                                            double robust_kernel_size) {
  robust_kernel_name_ = robust_kernel_name;
  robust_kernel_size_ = robust_kernel_size;
  need_robust_kernel_ = true;
}

void G2oGraphOptimizer::AddSE3Edge(int vertex_index1, int vertex_index2,
                                   const Eigen::Isometry3d &relative_pose,
                                   const Eigen::VectorXd noise) {
  Eigen::MatrixXd information_matrix =
      CalculateSE3EdgeInformationMatrix(noise);
  g2o::VertexSE3* v1 =
      dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index1));
  g2o::VertexSE3* v2 =
      dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index2));
  g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph_ptr_->addEdge(edge);
  if (need_robust_kernel_) {
      AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
  }
}

Eigen::MatrixXd G2oGraphOptimizer::CalculateSE3EdgeInformationMatrix(
    Eigen::VectorXd noise) {
  Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
  information_matrix = CalculateDiagMatrix(noise);
  return information_matrix;
}

void G2oGraphOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge *edge,
                                        const std::string &kernel_type,
                                        double kernel_size) {
  if (kernel_type == "NONE") {
      return;
  }

  g2o::RobustKernel *kernel = robust_kernel_factory_->construct(kernel_type);
  if (kernel == nullptr) {
      LOG(ERROR) << "warning : invalid robust kernel type: "
                 << kernel_type << std::endl;
      return;
  }

  kernel->setDelta(kernel_size);
  edge->setRobustKernel(kernel);
}

Eigen::MatrixXd G2oGraphOptimizer::CalculateDiagMatrix(Eigen::VectorXd noise) {
  Eigen::MatrixXd information_matrix =
      Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
  for (int i = 0; i < noise.rows(); i++) {
    // information matrix diagnal component is the reciprocal of noise value
    information_matrix(i, i) /= noise(i);
  }
  return information_matrix;
}

void G2oGraphOptimizer::AddSE3PriorXYZEdge(int se3_vertex_index,
                                           const Eigen::Vector3d &xyz,
                                           Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
    g2o::VertexSE3 *v_se3 =
        dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
}

void G2oGraphOptimizer::AddSE3PriorQuaternionEdge(int se3_vertex_index,
        const Eigen::Quaterniond &quat,
        Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix =
        CalculateSE3PriorQuaternionEdgeInformationMatrix(noise);
    g2o::VertexSE3 *v_se3 =
        dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorQuat *edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
}

// TODO(nxu): Orientation measurement information matrix to be added
Eigen::MatrixXd G2oGraphOptimizer::
    CalculateSE3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise) {
  Eigen::MatrixXd information_matrix;
  return information_matrix;
}
}  // namespace lidar_slam
