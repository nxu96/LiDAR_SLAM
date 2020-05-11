/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-10 22:18:49
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-10 22:47:38
 * @Description: File management implementation, file R/W
 */
#include "tools/file_manager.h"
#include <glog/logging.h>
#include <boost/filesystem.hpp>
namespace lidar_slam {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
  ofs.open(file_path.c_str(), std::ios::app);
  if (!ofs) {
    LOG(WARNING) << "[Create File] Unable to create this file: "
                 << file_path << std::endl;
    return false;
  }
  return true;
}

bool FileManager::CreateDirectory(std::string dir_path) {
  if (!boost::filesystem::is_directory(dir_path)) {
    boost::filesystem::create_directory(dir_path);
  }
  if (!boost::filesystem::is_directory(dir_path)) {
    LOG(WARNING) << "[Create Dir] Unable to create this directory: "
                 << dir_path << std::endl;
    return false;
  }
  return true;
}
}  // namespace lidar_slam
