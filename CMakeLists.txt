cmake_minimum_required(VERSION 2.8.3)
project(lidar_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
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
  message_generation
  std_srvs
)

add_service_files(
  FILES
  saveMap.srv
  optimizeMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(ALL_TARGET_LIBRARIES "")
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR})
# message(${PROJECT_SOURCE_DIR})

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/g2o.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_definition.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cc")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cc")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
# message(${NODE_SRCS})
# message("test!!!")
# message(${THIRD_PARTY_SRCS})
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})
# message(${ALL_SRCS})

add_executable(test_frame_node src/apps/test_frame_node.cc ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(pre_process_node src/apps/pre_process_node.cc ${ALL_SRCS})
add_dependencies(pre_process_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pre_process_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/apps/front_end_node.cc ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(back_end_node src/apps/back_end_node.cc ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS} optimizeMap_gencpp)
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(viewer_node src/apps/viewer_node.cc ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})