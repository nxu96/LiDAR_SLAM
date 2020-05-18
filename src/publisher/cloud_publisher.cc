/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 10:21:53
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 17:32:05
 * @Description: Point CLoud Publisher source file
 */

#include "publisher/cloud_publisher.h"
#include <glog/logging.h>

namespace lidar_slam {
CloudPublisher::CloudPublisher(const ros::NodeHandle& nh,
    std::string topic_name, size_t buff_sze,
    std::string frame_id) : nh_(nh), frame_id_(frame_id) {
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_sze);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR& cloud_ptr_input,
                             double time) {
  ros::Time ros_time(static_cast<float>(time));
  PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR& cloud_ptr_input) {
  ros::Time ros_time = ros::Time::now();
  PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::PublishData(CloudData::CLOUD_PTR& cloud_ptr_input,
                                 ros::Time ros_time) {
  sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
  cloud_ptr_output->header.stamp = ros_time;
  cloud_ptr_output->header.frame_id = frame_id_;
  pub_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
  return pub_.getNumSubscribers() != 0;
}

}  // namespace lidar_slam
