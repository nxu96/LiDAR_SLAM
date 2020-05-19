/*
 * @Author: Ning Xu
 * @Email: nxu@umich.edu
 * @Date: 2020-05-18 21:16:16
 * @Last Modified by: Ning Xu
 * @Last Modified time: 2020-05-18 21:21:17
 * @Description: Timer used for code profiling
 */
#ifndef LIDAR_SLAM_INCLUDE_TOOLS_TIMER_H_
#define LIDAR_SLAM_INCLUDE_TOOLS_TIMER_H_
#include <ctime>
#include <cstdlib>
#include <chrono>

namespace lidar_slam {
class Timer {
 public:
  Timer() {
    Tic();
  }

  void Tic() {
    start = std::chrono::system_clock::now();
  }

  double Toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> time_spent = end - start;
    start = std::chrono::system_clock::now();
    return time_spent.count();
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
}  // namespace lidar_slam
#endif  // LIDAR_SLAM_INCLUDE_TOOLS_TIMER_H_
