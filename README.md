# lidar_SLAM
LIDAR SLAM for Autonomous Vehicles

## Building command

```
catkin_make --only-pkg-with-deps lidar_slam
```

## TO-DO List

- [x] Kitti dataset preparation
- [x] Project framework design with ROS
- [x] Front end lidar odometry with Lidar NDT and voxel filter
- [x] Front end refactoring
- [x] Sensors time synchronization algorithm
- [ ] Add point cloud visualizer based on PCL
- [x] Lidar odometry accuracy evaluation with evo
- [ ] Point cloud distortion correction/compensation algorithm
- [ ] Module design and refactoring
  * Data pre-process
  * Front end lidar odometry
  * Back end graph optimization
  * Loop closure
  * Visualization
- [ ] Back end optimization with g2o and Ceres
- [ ] Loop closure
- [ ] Front end extension with A-LOAM
- [ ] Localization with global map



## Test Environment

* Ubuntu 18.04
* ROS Melodic
* PCL 1.8



