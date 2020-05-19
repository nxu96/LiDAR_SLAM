/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-18 21:29:11
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 21:34:53
 * @Description: Graph Optimizer Abstract class implementation
 */
#include "models/graph_optimizer/interface_graph_optimizer.h"

namespace lidar_slam {
void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iter_num) {
  max_iterations_num_ = max_iter_num;
}
}  // namespace lidar_slam
