/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-07 19:44:44
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 10:59:43
 * @Description: Front End Node
 */

#include <ros/ros.h>
#include <glog/logging.h>
#include <memory>
#include "global_definition/global_definition.h"
#include "mapping/front_end/front_end_flow.h"

using namespace lidar_slam;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = lidar_slam::WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "front_end_node");
  ros::NodeHandle nh;

  std::shared_ptr<FrontEndFlow> front_end_flow_ptr =
      std::make_shared<FrontEndFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    front_end_flow_ptr->Run();

    rate.sleep();
  }
  return 0;
}
