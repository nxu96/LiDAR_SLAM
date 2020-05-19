/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-18 11:03:40
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-19 12:55:21
 * @Description: BackEnd Node implementation
 */
#include <ros/ros.h>
#include <glog/logging.h>
#include <lidar_slam/optimizeMap.h>
#include "global_definition/global_definition.h"
#include "mapping/back_end/back_end_flow.h"

using namespace lidar_slam;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request &req,
                           optimizeMap::Response &res) {
  _need_optimize_map = true;
  res.succeed = true;
  return res.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "back_end_node");
  ros::NodeHandle nh;
  ros::ServiceServer service =
    nh.advertiseService("optimize_map", optimize_map_callback);
  _back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    _back_end_flow_ptr->Run();

    if (_need_optimize_map) {
      _need_optimize_map = false;
      _back_end_flow_ptr->ForceOptimize();
    }

    rate.sleep();
  }

  return 0;
}
