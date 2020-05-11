/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-10 22:11:37
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-10 23:01:57
 * @Description: File manager header file, defining some file R/W related ops
 */
#ifndef LIDAR_SLAM_INCLUDE_TOOLS_FILE_MANAGER_H_
#define LIDAR_SLAM_INCLUDE_TOOLS_FILE_MANAGER_H_

#include <string>
#include <iostream>
#include <fstream>

namespace lidar_slam {
class FileManager {
 public:
  static bool CreateFile(std::ofstream& ofs, std::string file_path);

  static bool CreateDirectory(std::string dir_path);
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_TOOLS_FILE_MANAGER_H_
