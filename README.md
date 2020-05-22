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
- [x] Point cloud distortion correction/motion compensation algorithm
- [x] Module design and refactoring
  * Data pre-process
  * Front end lidar odometry
  * Back end graph optimization
  * Loop closure
  * Visualization
- [x] Back end optimization with g2o and Ceres
- [ ] Loop closure
- [ ] Front end extension with A-LOAM
- [ ] Localization with global map
- [ ] IMU state model derivation





## Test Environment

* Ubuntu 18.04
* ROS Melodic
* PCL 1.8

### G2O Installation

use the zip file in the `third_party` directory

```
cd build
cmake ..
make -j8
sudo make install
```

> One error I met while building this project with G2O: 
>
> cholmod library not found. 
> It is solved by installing the dependency `libsuitesparse-dev`and **re-install the g2o library**
>
> `sudo apt-get install libsuitesparse-dev`





## Paper TO-DO List

1. Robust and Precise Vehicle Localization Based on Multi-Sensor Fusion in Diverse City Scenes
2. LiDAR Inertial Odometry Aided Robust LiDAR Localization System in Changing City Scenes

