cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_align)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  rosbag
)

find_package(PCL REQUIRED QUIET)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_align
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
 DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(transform_trajectories src/transform_trajectories.cpp)

target_link_libraries(transform_trajectories
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(
#   TARGETS transform_trajectories
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
