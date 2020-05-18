/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 14:38:01
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 15:37:51
 * @Description: Subscribe  the multiple key frames data
 */
#ifndef LIDAR_SLAM_INCLUDE_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_H_
#define LIDAR_SLAM_INCLUDE_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_H_
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <deque>
#include <string>

#include "sensor_data/key_frame.h"

namespace lidar_slam {
class KeyFramesSubscriber {
 public:
  KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name,
                     std::size_t buff_size);
  KeyFramesSubscriber() = default;
  void ParseData(std::deque<KeyFrame>& deque_key_frames);

 private:
  void msg_callback(
      const nav_msgs::Path::ConstPtr& key_frames_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  std::deque<KeyFrame> new_key_frame_;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_H_
