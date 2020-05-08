/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-07 19:44:44
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-08 17:057
 * @Description: Front End Node
 */

#include <ros/ros.h>
#include <memory>
#include <lidar_slam/saveMap.h>
#include "glog/logging.h"
#include "global_definition/global_definition.h"
#include "front_end/front_end_flow.h"

using namespace lidar_slam;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

bool save_map_callback(saveMap::Request& req, saveMap::Response& res) {
  res.succeed = _front_end_flow_ptr->SaveMap();
  _front_end_flow_ptr->PublishGlobalMap();
  return res.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = lidar_slam::WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "front_end_node");
  ros::NodeHandle nh;
  ros::ServiceServer service =
    nh.advertiseService("save_map", save_map_callback);

  _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    _front_end_flow_ptr->Run();
    rate.sleep();
  }
  return 0;
}
