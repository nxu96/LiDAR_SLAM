cmake_minimum_required(VERSION 2.8.3)
project(lidar_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -o3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
)
set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_definition.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cc")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cc")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
message(${NODE_SRCS})
message("test!!!")
message(${THIRD_PARTY_SRCS})
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(test_frame_node src/test_frame_node.cc ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})