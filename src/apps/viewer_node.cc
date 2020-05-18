/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-18 10:40:11
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 11:42:12
 * @Description: ViewerNode Implementation
 */
#include <ros/ros.h>
#include <glog/logging.h>
#include <memory>
#include <lidar_slam/saveMap.h>
#include "global_definition/global_definition.h"
#include "mapping/viewer/viewer_flow.h"

using namespace lidar_slam;

std::shared_ptr<ViewerFlow> _viewer_flow_ptr;
bool _need_save_map = false;

bool save_map_callback(saveMap::Request& req, saveMap::Response& res) {
  _need_save_map = true;
  res.succeed = true;
  return res.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = lidar_slam::WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "viewer_node");
  ros::NodeHandle nh;
  ros::ServiceServer service =
    nh.advertiseService("save_map", save_map_callback);

  _viewer_flow_ptr = std::make_shared<ViewerFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    _viewer_flow_ptr->Run();

    if (_need_save_map) {
      _need_save_map = false;
      _viewer_flow_ptr->SaveMap();
    }

    rate.sleep();
  }
  return 0;
}
