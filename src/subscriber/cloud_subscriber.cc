/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 11:25:13
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 13:05:00
 * @Description: Point cloud subscriber class source file
 */

#include "subscriber/cloud_subscriber.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "glog/logging.h"

namespace lidar_slam {
CloudSubscriber::CloudSubscriber(const ros::NodeHandle& nh, std::string topic,
                                 size_t buff_size) : nh_(nh) {
  sub_ = nh_.subscribe(topic, buff_size, &CloudSubscriber::OnPointCloud, this);
}

void CloudSubscriber::ParseData(std::deque<CloudData>* deque_cloud_data) {
  if (!new_cloud_data_.empty()) {
    // if we have new data in the buffer
    deque_cloud_data->insert(deque_cloud_data->end(),
                             new_cloud_data_.begin(), new_cloud_data_.end());
    new_cloud_data_.clear();
  }
}

void CloudSubscriber::OnPointCloud(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr) {
  CloudData data;
  data.time_ = cloud_ptr->header.stamp.toSec();
  // copy cloud data from ros message
  pcl::fromROSMsg(*cloud_ptr, *(data.cloud_ptr_));
  new_cloud_data_.push_back(data);
}

}  // namespace lidar_slam
