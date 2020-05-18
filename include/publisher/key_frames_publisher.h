/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 14:07:51
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 16:53:30
 * @Description: Key frameS publisher header file
 */

#ifndef LIDAR_SLAM_INCLUDE_PUBLISHER_KEY_FRAMES_PUBLISHER_H_
#define LIDAR_SLAM_INCLUDE_PUBLISHER_KEY_FRAMES_PUBLISHER_H_
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string>
#include <deque>
#include "sensor_data/key_frame.h"
namespace lidar_slam {
class KeyFramesPublisher {
 public:
  KeyFramesPublisher(ros::NodeHandle& nh, std::string topic_name,
                     std::string frame_id, size_t buff_size);
  KeyFramesPublisher() = default;
  void Publish(const std::deque<KeyFrame>& key_frames);
  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string frame_id_ = "";
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_PUBLISHER_KEY_FRAMES_PUBLISHER_H_
