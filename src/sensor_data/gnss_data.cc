/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 12:48:18
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-09 13:43:57
 * @Description: GNSS Data source file
 */

#include "sensor_data/gnss_data.h"
#include <glog/logging.h>

// we have to initialize the static member outside the class
bool lidar_slam::GNSSData::origin_position_initialized_ = false;
GeographicLib::LocalCartesian lidar_slam::GNSSData::geo_converter_;

namespace lidar_slam {

void GNSSData::InitOriginPosition() {
  geo_converter_.Reset(latitude_, longitude_, altitude_);
  origin_position_initialized_ = true;
}

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData,
  std::deque<GNSSData>& SyncedData, double sync_time) {
  while (UnsyncedData.size() >= 2) {
    if (UnsyncedData.front().time_ > sync_time) {
      return false;
    }
    if (UnsyncedData.back().time_ < sync_time) {
      return false;
    }
    if (UnsyncedData.at(1).time_ < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }

    // huge time diff ----> lost data
    if (sync_time - UnsyncedData.front().time_ > 0.2) {
      UnsyncedData.pop_front();
      // break;
      return false;
    }
    if (UnsyncedData.at(1).time_ - sync_time > 0.2) {
      // UnsyncedData.pop_front();
      // break;
      return false;
    }

    break;
  }
  if (UnsyncedData.size() < 2) {
    return false;
  }
  GNSSData front_data = UnsyncedData.at(0);
  GNSSData back_data = UnsyncedData.at(1);
  GNSSData synced_data;

  double front_scale = (back_data.time_ - sync_time) /
    (back_data.time_ - front_data.time_);
  double back_scale = (sync_time - front_data.time_) /
    (back_data.time_ - front_data.time_);

  synced_data.time_ = sync_time;
  synced_data.status_ = back_data.status_;
  synced_data.service_ = back_data.service_;

  synced_data.longitude_ = front_data.longitude_ * front_scale
                           + back_data.longitude_ * back_scale;
  synced_data.latitude_ = front_data.latitude_ * front_scale
                           + back_data.latitude_ * back_scale;
  synced_data.altitude_ = front_data.altitude_ * front_scale
                           + back_data.altitude_ * back_scale;
  synced_data.local_E_ = front_data.local_E_ * front_scale
                           + back_data.local_E_ * back_scale;
  synced_data.local_N_ = front_data.local_N_ * front_scale
                           + back_data.local_N_ * back_scale;
  synced_data.local_U_ = front_data.local_U_ * front_scale
                           + back_data.local_U_ * back_scale;

  SyncedData.push_back(synced_data);

  return true;
}

void GNSSData::UpdateXYZ() {
  if (!origin_position_initialized_) {
    LOG(WARNING) << "GeoConverter has not set origin position";
  }
  geo_converter_.Forward(latitude_, longitude_, altitude_,
                         local_E_, local_N_, local_U_);
}

}  // namespace lidar_slam
