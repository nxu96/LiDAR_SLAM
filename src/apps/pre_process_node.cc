/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-18 11:06:28
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 11:13:04
 * @Description: PreProcess Node Implementation
 */
#include <ros/ros.h>
#include <glog/logging.h>

#include "global_definition/global_definition.h"
#include "pre_process/pre_process_flow.h"

using namespace lidar_slam;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "pre_process_node");
  ros::NodeHandle nh;

  std::shared_ptr<PreProcessFlow> pre_process_flow_ptr =
      std::make_shared<PreProcessFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    pre_process_flow_ptr->Run();

    rate.sleep();
  }

  return 0;
}
