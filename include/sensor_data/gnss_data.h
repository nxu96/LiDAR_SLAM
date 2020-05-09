/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 12:36:53
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-09 12:00:39
 * @Description: Gnss data class
 */

#ifndef LIDAR_SLAM_INCLUDE_SENSOR_DATA_GNSS_DATA_H_
#define LIDAR_SLAM_INCLUDE_SENSOR_DATA_GNSS_DATA_H_
#include <deque>
#include "Geocentric/LocalCartesian.hpp"

namespace lidar_slam {
class GNSSData {
 public:
  GNSSData() = default;
  void InitOriginPosition();
  void UpdateXYZ();

 public:
  double time_ = 0.0;
  double longitude_ = 0.0;
  double latitude_ = 0.0;
  double altitude_ = 0.0;
  double local_E_ = 0.0;
  double local_N_ = 0.0;
  double local_U_ = 0.0;
  int status_ = 0;
  int service_ = 0;
 
 public:
  static bool SyncData(std::deque<GNSSData>& UnsyncedData,
    std::deque<GNSSData>& SyncedData, double sync_time);
  
 private:
  static GeographicLib::LocalCartesian geo_converter_;
  static bool origin_position_initialized_;
};
}  // namespace lidar_slam

#endif  // LIDAR_SLAM_INCLUDE_SENSOR_DATA_GNSS_DATA_H_
