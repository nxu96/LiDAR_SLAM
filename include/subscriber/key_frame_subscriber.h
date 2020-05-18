/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 14:38:01
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 15:10:27
 * @Description: Subscribe  the key frame data
 */
#ifndef LIDAR_SLAM_INCLUDE_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_H_
#define LIDAR_SLAM_INCLUDE_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_H_
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <deque>
#include <string>
#include "sensor_data/key_frame.h"

namespace lidar_slam {
class KeyFrameSubscriber {
 public:
  KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name,
                     std::size_t buff_size);
  KeyFrameSubscriber() = default;
  void ParseData(std::deque<KeyFrame>& key_frame_buff);

 private:
  void msg_callback(
      const geometry_msgs::PoseStampedConstPtr& key_frame_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  std::deque<KeyFrame> new_key_frame_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_H_
