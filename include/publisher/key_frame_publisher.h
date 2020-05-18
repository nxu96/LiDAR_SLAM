/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-17 14:07:51
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-17 17:35:12
 * @Description: Key frame publisher header file
 */

#ifndef LIDAR_SLAM_INCLUDE_PUBLISHER_KEY_FRAME_PUBLISHER_H_
#define LIDAR_SLAM_INCLUDE_PUBLISHER_KEY_FRAME_PUBLISHER_H_
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>
#include "sensor_data/key_frame.h"
namespace lidar_slam {
class KeyFramePublisher {
 public:
  KeyFramePublisher(ros::NodeHandle& nh, std::string topic_name,
                    std::string frame_id, size_t buff_size);
  KeyFramePublisher() = default;
  void Publish(KeyFrame& key_frame);
  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string frame_id_ = "";
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_PUBLISHER_KEY_FRAME_PUBLISHER_H_
