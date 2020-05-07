/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-04-22 12:48:18
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-04-22 12:53:48
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

void GNSSData::UpdateXYZ() {
  if (!origin_position_initialized_) {
    LOG(WARNING) << "GeoConverter has not set origin position";
  }
  geo_converter_.Forward(latitude_, longitude_, altitude_,
                         local_E_, local_N_, local_U_);
}

}  // namespace lidar_slam
