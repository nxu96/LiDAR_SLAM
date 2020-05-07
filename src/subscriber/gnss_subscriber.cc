/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 18:30:25
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 18:36:05
 * @Description: Description
 */

#include "subscriber/gnss_subscriber.h"

namespace lidar_slam {
GNSSSubscriber::GNSSSubscriber(const ros::NodeHandle& nh, std::string topic,
    size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic, buff_size, &GNSSSubscriber::OnGNSS, this);
}

void GNSSSubscriber::ParseData(std::deque<GNSSData>* deque_gnss_data) {
  if (!new_gnss_data_.empty()) {
    deque_gnss_data->insert(deque_gnss_data->end(), new_gnss_data_.begin(),
                           new_gnss_data_.end());
    new_gnss_data_.clear();
  }
}

void GNSSSubscriber::OnGNSS(const sensor_msgs::NavSatFixConstPtr& nav_msg_ptr) {
  GNSSData gnss_data;

  gnss_data.time_ = nav_msg_ptr->header.stamp.toSec();
  gnss_data.latitude_ = nav_msg_ptr->latitude;
  gnss_data.longitude_ = nav_msg_ptr->longitude;
  gnss_data.altitude_ = nav_msg_ptr->altitude;
  gnss_data.status_ = nav_msg_ptr->status.status;
  gnss_data.service_ = nav_msg_ptr->status.service;

  new_gnss_data_.push_back(gnss_data);
}

}  // namespace lidar_slam
