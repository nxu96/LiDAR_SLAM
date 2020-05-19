/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-18 22:01:11
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 22:37:38
 * @Description: Prior edge with GNSS position measurement header file
 */
#ifndef LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_H_
#define LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_H_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
class EdgeSE3PriorXYZ :
    public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // macro for memory alignment??
  // g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> template class
  // NOTE: This is the constructor for this derived class
  EdgeSE3PriorXYZ() :
      g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {}

  void computeError() override {
    const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

    Eigen::Vector3d estimate = v1->estimate().translation();
    _error = estimate - _measurement;  // update the error using est and measure
  }

  void setMeasurement(const Eigen::Vector3d& m) override {
    _measurement = m;
  }

  bool read(std::istream& is) override {
    Eigen::Vector3d v;
    is >> v(0) >> v(1) >> v(2);
    setMeasurement(Eigen::Vector3d(v));

    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }
    return true;
  }

  bool write(std::ostream& os) const override {
    Eigen::Vector3d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " ";
    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      }
    }
    return os.good();
  }
};
}  // namespace g2o

#endif  // LIDAR_SLAM_INCLUDE_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_H_
